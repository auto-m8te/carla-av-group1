"""LiDAR obstacle detection (E3 Obstacle Detection, owner: Abir).

Pure numpy, NO CARLA dependency, so it is unit-tested on synthetic point clouds.
`perception/lidar_sensor.py` feeds it real CARLA LiDAR sweeps.

Input : point cloud in the VEHICLE frame (x forward, y right, z up, metres,
        origin on the ground under the car centre).
Outputs:
    detect_obstacle -> Detection(ahead, distance, ...); `distance` is the agreed
                       `obstacle_distance` interface: bumper-to-obstacle gap, or None.
    lane_occupied   -> is the lane we would swerve into blocked? (feeds the
                       avoidance module's `adjacent_lane_clear`)

Pipeline per sweep (shared by both):
    1. Range gate      keep points in the x-window of interest; drop our own car body
    2. Band            keep points within a lateral band around a centreline (our lane
                       ahead, or the adjacent lane), from the map path or a yaw-rate arc
    3. Ground removal  per 2 m range bin, estimate road height from points near the
                       band (lowest point, limited to a plausible road grade) and drop
                       points near it; also drop overhead points (signs, bridges, trees)
    4. Cluster check   group of >= min_points within cluster_depth metres
                       -> rejects isolated noise returns
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class LidarDetectionConfig:
    max_range: float = 60.0         # m, matches the avoidance module's detection range
    ego_half_length: float = 2.4    # m, car centre -> front bumper
    ego_half_width: float = 1.0     # m
    path_margin: float = 0.3        # m, extra lateral clearance either side
    min_height: float = 0.35        # m above estimated road -> counts as obstacle
    max_height: float = 2.5         # m above road; higher = overhead, ignored
    ground_bin: float = 2.0         # m, range bin size for road-height estimate
    max_road_grade: float = 0.10    # road can rise at most 10% per metre between bins
    ground_lateral: float = 3.0     # m beyond the band used for road height (not the
                                    # whole sweep: terrain below an elevated road is lower)
    min_points: int = 3             # points needed to accept a detection
    cluster_depth: float = 1.0      # m, depth window those points must fall into
    ego_body_margin: float = 0.3    # m around our own car whose returns are ignored


@dataclass(frozen=True)
class Detection:
    ahead: bool                                 # "obstacle ahead" flag
    distance: Optional[float]                   # gap in metres (None = path clear)
    nearest_point: Optional[tuple[float, float, float]] = None  # vehicle frame, for debug draw
    num_points: int = 0                         # corridor points in the accepted cluster


CLEAR = Detection(ahead=False, distance=None)
PATH_END_MARGIN = 1.0  # m the band may extend past the first/last path point


def _grade_limited(bin_min: np.ndarray, start: int, step: int, max_step: float,
                   ground: np.ndarray) -> None:
    """Walk bins away from the car (index `start`), filling `ground` in place."""
    prev = 0.0  # road under the ego car
    i = start
    while 0 <= i < bin_min.size:
        lowest = bin_min[i]
        g = prev + max_step if not np.isfinite(lowest) else min(lowest, prev + max_step)
        ground[i] = max(g, prev - max_step)  # road cannot drop off a cliff either
        prev = ground[i]
        i += step


def _road_height(x: np.ndarray, z: np.ndarray, near_band: np.ndarray, x_min: float,
                 x_max: float, cfg: LidarDetectionConfig) -> np.ndarray:
    """Road height under each point, from the lowest return near the band in its bin.

    Starting at the car and walking outwards (forwards and backwards), the estimate may
    only change by `max_road_grade` per metre, so a bin that only contains car body
    points (road hidden) cannot be mistaken for road.
    """
    n_bins = int(np.ceil((x_max - x_min) / cfg.ground_bin)) + 1
    idx = np.clip(((x - x_min) / cfg.ground_bin).astype(int), 0, n_bins - 1)
    bin_min = np.full(n_bins, np.inf)
    np.minimum.at(bin_min, idx[near_band], z[near_band])
    ground = np.empty(n_bins)
    car_bin = int(np.clip((0.0 - x_min) / cfg.ground_bin, 0, n_bins - 1))
    max_step = cfg.max_road_grade * cfg.ground_bin
    _grade_limited(bin_min, car_bin, +1, max_step, ground)
    _grade_limited(bin_min, car_bin, -1, max_step, ground)
    return ground[idx]


def _band_centre(x: np.ndarray, curvature: float, path: Optional[np.ndarray]) -> np.ndarray:
    """Lateral centre of the band at each forward distance x."""
    if path is None:
        return 0.5 * curvature * x ** 2  # small-angle arc: y = x^2 / (2R)
    p = np.asarray(path, dtype=float)
    if p.ndim != 2 or p.shape[1] != 2 or p.shape[0] < 2:
        raise ValueError(f"path must be (M>=2, 2), got {p.shape}")
    p = p[np.argsort(p[:, 0])]
    return np.interp(x, p[:, 0], p[:, 1])  # held constant beyond the ends


# Kept for callers/tests that use the old name.
_corridor_centre = _band_centre


def _as_points(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError(f"points must be (N, 3+), got {pts.shape}")
    return pts


def _band_clusters(
    pts: np.ndarray, cfg: LidarDetectionConfig, half_width: float, x_min: float,
    x_max: float, curvature: float, path: Optional[np.ndarray],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Obstacle points inside the band, sorted by x, plus cluster sizes.

    Returns (cx, cy, cz, counts, hits, indices): indices map back to input points; counts[i] = points within cluster_depth ahead of
    point i; hits = indices whose cluster is big enough, nearest first.
    """
    empty = np.empty(0)
    if path is not None and len(path) >= 2:
        # Only look as far as the path reaches: where the road turns away, the
        # straight-ahead continuation is buildings/trees, not our lane.
        px = np.asarray(path, dtype=float)[:, 0]
        x_min = max(x_min, float(px.min()) - PATH_END_MARGIN)
        x_max = min(x_max, float(px.max()) + PATH_END_MARGIN)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    own_body = ((np.abs(x) < cfg.ego_half_length + cfg.ego_body_margin)
                & (np.abs(y) < cfg.ego_half_width + cfg.ego_body_margin))
    keep = (x > x_min) & (x < x_max) & ~own_body
    if not keep.any():
        return empty, empty, empty, empty, empty.astype(int), empty.astype(int)
    x, y, z = x[keep], y[keep], z[keep]

    lateral = np.abs(y - _band_centre(x, curvature, path))
    height = z - _road_height(x, z, lateral < half_width + cfg.ground_lateral, x_min, x_max, cfg)
    mask = (height > cfg.min_height) & (height < cfg.max_height) & (lateral < half_width)
    if not mask.any():
        return empty, empty, empty, empty, empty.astype(int), empty.astype(int)

    order = np.argsort(x[mask])
    cx, cy, cz = x[mask][order], y[mask][order], z[mask][order]
    counts = np.searchsorted(cx, cx + cfg.cluster_depth, side="right") - np.arange(cx.size)
    indices = np.flatnonzero(keep)[mask][order]
    return cx, cy, cz, counts, np.flatnonzero(counts >= cfg.min_points), indices


def detect_obstacle(
    points: np.ndarray,
    cfg: LidarDetectionConfig,
    curvature: float = 0.0,
    path: Optional[np.ndarray] = None,
) -> Detection:
    """Find the nearest obstacle in the car's path.

    points    : (N, 3+) array, vehicle frame. Extra columns (intensity) are ignored.
    path      : (M, 2) predicted path centreline (x, y) in the vehicle frame, e.g. the
                lane ahead from the map/planner. Preferred: it follows curves exactly.
    curvature : used only without `path`; 1/radius, + = turning right.
    """
    pts = _as_points(points)
    if pts.shape[0] == 0:
        return CLEAR
    cx, cy, cz, counts, hits, _ = _band_clusters(
        pts, cfg, cfg.ego_half_width + cfg.path_margin,
        cfg.ego_half_length, cfg.ego_half_length + cfg.max_range, curvature, path,
    )
    if hits.size == 0:
        return CLEAR
    i = hits[0]
    return Detection(
        ahead=True,
        distance=float(cx[i] - cfg.ego_half_length),
        nearest_point=(float(cx[i]), float(cy[i]), float(cz[i])),
        num_points=int(counts[i]),
    )


def lane_occupied(
    points: np.ndarray,
    cfg: LidarDetectionConfig,
    lane_path: np.ndarray,
    lane_width: float,
    behind: float = 15.0,
    ahead: float = 60.0,
) -> bool:
    """Is anything standing in the lane described by `lane_path`?

    lane_path : (M, 2) centreline (x, y) of the lane to check, vehicle frame; should
                span from `behind` metres behind the car to `ahead` metres in front.
    Checks from `behind` metres behind the car centre to `ahead` metres in front, over
    the full lane width (an overtake needs the whole lane, not just our car's width).
    """
    return bool(lane_occupied_points(points, cfg, lane_path, lane_width, behind, ahead).any())


def lane_occupied_points(
    points: np.ndarray,
    cfg: LidarDetectionConfig,
    lane_path: np.ndarray,
    lane_width: float,
    behind: float = 15.0,
    ahead: float = 60.0,
) -> np.ndarray:
    """Boolean mask over input points belonging to accepted lane-band clusters."""
    pts = _as_points(points)
    mask = np.zeros(len(pts), dtype=bool)
    cx, _, _, _, hits, indices = _band_clusters(
        pts, cfg, lane_width / 2.0, -behind, ahead, 0.0, lane_path,
    )
    if hits.size:
        # Count interval coverage without allocating a points-by-clusters matrix.
        coverage = np.zeros(cx.size + 1, dtype=int)
        starts = np.searchsorted(cx, cx[hits], side="left")
        ends = np.searchsorted(cx, cx[hits] + cfg.cluster_depth, side="right")
        np.add.at(coverage, starts, 1)
        np.add.at(coverage, ends, -1)
        mask[indices] = np.cumsum(coverage[:-1]) > 0
    return mask
