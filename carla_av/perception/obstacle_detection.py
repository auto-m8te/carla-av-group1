"""LiDAR obstacle detection (E3 Obstacle Detection, owner: Abir).

Pure numpy, NO CARLA dependency, so it is unit-tested on synthetic point clouds.
`perception/lidar_sensor.py` feeds it real CARLA LiDAR sweeps.

Input : point cloud in the VEHICLE frame (x forward, y right, z up, metres,
        origin on the ground under the car centre).
Output: `Detection(ahead, distance, ...)` where `distance` is the agreed
        `obstacle_distance` interface: bumper-to-obstacle gap in metres, or None.

Pipeline per sweep:
    1. Range gate      keep points between the ego front bumper and max_range
    2. Ground removal  per 2 m range bin, estimate road height (lowest point,
                       limited to a plausible road grade) and drop points near it;
                       also drop overhead points (signs, bridges, trees)
    3. Path corridor   keep points inside the car's predicted path (lane ahead from
                       the map, or a yaw-rate arc as fallback): ego width +
                       margin either side
    4. Cluster check   nearest group of >= min_points within cluster_depth metres
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
    ground_lateral: float = 3.0     # m beyond the corridor used for road height (not the
                                    # whole sweep: terrain below an elevated road is lower)
    min_points: int = 3             # points needed to accept a detection
    cluster_depth: float = 1.0      # m, depth window those points must fall into


@dataclass(frozen=True)
class Detection:
    ahead: bool                                 # "obstacle ahead" flag
    distance: Optional[float]                   # gap in metres (None = path clear)
    nearest_point: Optional[tuple[float, float, float]] = None  # vehicle frame, for debug draw
    num_points: int = 0                         # corridor points in the accepted cluster


CLEAR = Detection(ahead=False, distance=None)


def _road_height(x: np.ndarray, z: np.ndarray, near_path: np.ndarray,
                 cfg: LidarDetectionConfig) -> np.ndarray:
    """Road height under each point, from the lowest return near the path in its range bin.

    The estimate may only rise by `max_road_grade` per metre from bin to bin, so a
    bin that only contains car body points (road hidden) cannot be mistaken for road.
    """
    n_bins = int(np.ceil((cfg.ego_half_length + cfg.max_range) / cfg.ground_bin)) + 1
    idx = np.clip((x / cfg.ground_bin).astype(int), 0, n_bins - 1)
    bin_min = np.full(n_bins, np.inf)
    np.minimum.at(bin_min, idx[near_path], z[near_path])
    ground = np.empty(n_bins)
    prev = 0.0  # road under the ego car
    max_step = cfg.max_road_grade * cfg.ground_bin
    for i in range(n_bins):
        lowest = bin_min[i]
        ground[i] = prev + max_step if not np.isfinite(lowest) else min(lowest, prev + max_step)
        ground[i] = max(ground[i], prev - max_step)  # road cannot drop off a cliff either
        prev = ground[i]
    return ground[idx]


def _corridor_centre(x: np.ndarray, curvature: float, path: Optional[np.ndarray]) -> np.ndarray:
    """Lateral centre of the predicted path at each forward distance x."""
    if path is None:
        return 0.5 * curvature * x ** 2  # small-angle arc: y = x^2 / (2R)
    p = np.asarray(path, dtype=float)
    if p.ndim != 2 or p.shape[1] != 2 or p.shape[0] < 2:
        raise ValueError(f"path must be (M>=2, 2), got {p.shape}")
    p = p[np.argsort(p[:, 0])]
    return np.interp(x, p[:, 0], p[:, 1])  # held constant beyond the ends


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
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError(f"points must be (N, 3+), got {pts.shape}")
    if pts.shape[0] == 0:
        return CLEAR

    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    in_range = (x > cfg.ego_half_length) & (x < cfg.ego_half_length + cfg.max_range)
    if not in_range.any():
        return CLEAR
    x, y, z = x[in_range], y[in_range], z[in_range]

    centre_y = _corridor_centre(x, curvature, path)
    lateral = np.abs(y - centre_y)
    corridor = cfg.ego_half_width + cfg.path_margin
    # Road height from points near the path (road beside an obstacle is still visible).
    height = z - _road_height(x, z, lateral < corridor + cfg.ground_lateral, cfg)
    mask = (height > cfg.min_height) & (height < cfg.max_height) & (lateral < corridor)
    if not mask.any():
        return CLEAR

    cx, cy, cz = x[mask], y[mask], z[mask]
    order = np.argsort(cx)
    cx, cy, cz = cx[order], cy[order], cz[order]
    # For each candidate i: how many points lie in [cx[i], cx[i] + cluster_depth]?
    counts = np.searchsorted(cx, cx + cfg.cluster_depth, side="right") - np.arange(cx.size)
    hits = np.flatnonzero(counts >= cfg.min_points)
    if hits.size == 0:
        return CLEAR

    i = hits[0]
    return Detection(
        ahead=True,
        distance=float(cx[i] - cfg.ego_half_length),
        nearest_point=(float(cx[i]), float(cy[i]), float(cz[i])),
        num_points=int(counts[i]),
    )
