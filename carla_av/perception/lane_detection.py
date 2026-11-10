"""Colour/edge lane detection and signed PD lane keeping, independent of CARLA."""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from collections import deque
from typing import Optional

import cv2
import numpy as np

log = logging.getLogger(__name__)
Line = tuple[float, float]  # x = a*y + b


@dataclass(frozen=True)
class LaneDetectionConfig:
    width: int = 800
    height: int = 450
    roi_bottom_y: float = 1.0
    roi_top_y: float = 0.52
    roi_top_width: float = 0.65
    roi_bottom_width: float = 1.0
    white_s_max: int = 39
    white_v_min: int = 181  # retained for configuration compatibility; white uses contrast
    yellow_h_min: int = 15
    yellow_h_max: int = 35
    yellow_s_min: int = 81
    yellow_v_min: int = 121
    canny_low: int = 50
    canny_high: int = 150
    hough_rho: float = 1.0
    hough_theta: float = math.pi / 180
    hough_threshold: int = 20
    min_line_length: int = 20
    max_line_gap: int = 40
    min_slope: float = 0.4
    lane_width_m: float = 3.5
    lookahead_fraction: float = 0.75


@dataclass(frozen=True)
class LaneEstimate:
    lateral_offset_m: Optional[float]
    heading_error_rad: Optional[float]
    confidence: float
    left: Optional[Line]
    right: Optional[Line]


def _roi(cfg: LaneDetectionConfig) -> np.ndarray:
    vertices = np.array([[(1-cfg.roi_bottom_width)/2, cfg.roi_bottom_y],
                     [(1-cfg.roi_top_width)/2, cfg.roi_top_y],
                     [(1+cfg.roi_top_width)/2, cfg.roi_top_y],
                     [(1+cfg.roi_bottom_width)/2, cfg.roi_bottom_y]]
                    )
    return (vertices * [cfg.width-1, cfg.height-1]).astype(np.int32)


def _segments(bgr: np.ndarray, cfg: LaneDetectionConfig) -> Optional[np.ndarray]:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    value = hsv[:, :, 2]
    background = cv2.GaussianBlur(value, (41, 41), 0)
    contrast = value.astype(np.int16) - background.astype(np.int16)
    white = np.uint8((contrast > 12) & (hsv[:, :, 1] <= cfg.white_s_max)) * 255
    yellow = cv2.inRange(hsv, (cfg.yellow_h_min, cfg.yellow_s_min, cfg.yellow_v_min),
                         (cfg.yellow_h_max, 255, 255))
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(white | yellow, cfg.canny_low, cfg.canny_high)
    edges |= cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 60, 140)
    mask = np.zeros(edges.shape, dtype=np.uint8)
    cv2.fillPoly(mask, [_roi(cfg)], 255)
    return cv2.HoughLinesP(edges & mask, cfg.hough_rho, cfg.hough_theta,
                           cfg.hough_threshold, minLineLength=cfg.min_line_length,
                           maxLineGap=cfg.max_line_gap)


def _fit_sides(segments: Optional[np.ndarray], cfg: LaneDetectionConfig, prior_width_px: Optional[float] = None
               ) -> tuple[Optional[Line], Optional[Line]]:
    sides = [[], []]
    if segments is not None:
        for x1, y1, x2, y2 in segments.reshape(-1, 4):
            dx, dy = float(x2-x1), float(y2-y1)
            if max(y1, y2) < cfg.height*.70 or abs(dy) < 12 or (dx != 0 and abs(dy/dx) < cfg.min_slope):
                continue
            a = dx/dy
            b = x1-a*y1
            bottom = a*(cfg.height-1)+b
            side = 0 if bottom < cfg.width/2 else 1
            if not -cfg.width*.6 < bottom < cfg.width*1.6:
                continue
            if (side == 0 and a >= 0) or (side == 1 and a <= 0):
                continue
            # Perspective boundaries converge near the horizon, not the skyline.
            horizon_x = a*(cfg.height*.38)+b
            if not cfg.width*.15 < horizon_x < cfg.width*.85:
                continue
            gx0, gz0 = _ground_point(a*(cfg.height-1)+b, cfg.height-1, cfg)
            gx1, gz1 = _ground_point(a*(cfg.height*.7)+b, cfg.height*.7, cfg)
            if abs((gx1-gx0)/(gz1-gz0)) > .5:
                continue
            sides[side].append((bottom, a, b, math.hypot(dx, dy)))
    choices = []
    for candidates in sides:
        if not candidates:
            choices.append([])
            continue
        # Cluster duplicate marking edges before comparing lane-width and heading consistency.
        clusters = []
        for candidate in sorted(candidates):
            if not clusters or candidate[0]-np.mean([c[0] for c in clusters[-1]]) > 45:
                clusters.append([])
            clusters[-1].append(candidate)
        clusters = [c for c in clusters if sum(v[3] for v in c) >= 35]
        if not clusters:
            choices.append([])
            continue
        choices.append([(tuple(np.average(np.array(c)[:, 1:3], axis=0, weights=np.array(c)[:, 3])), sum(v[3] for v in c)) for c in clusters])
    def properties(line):
        a, b = line
        x0, z0 = _ground_point(a*(cfg.height-1)+b, cfg.height-1, cfg)
        x1, z1 = _ground_point(a*cfg.height*.7+b, cfg.height*.7, cfg)
        slope = (x1-x0)/(z1-z0)
        return x0, slope, x0-slope*z0
    expected = prior_width_px if prior_width_px is not None else cfg.lane_width_m / _ground_point(cfg.width/2+1, cfg.height-1, cfg)[0]
    pairs = []
    for left, wl in choices[0]:
        for right, wr in choices[1]:
            xl, ml, bl = properties(left)
            xr, mr, br = properties(right)
            width = xr-xl
            lb = left[0]*(cfg.height-1)+left[1]
            rb = right[0]*(cfg.height-1)+right[1]
            pixel_width = rb-lb
            vy = (right[1]-left[1])/(left[0]-right[0])
            vx = left[0]*vy+left[1]
            if not (lb < cfg.width/2 < rb and .6*expected < pixel_width < 1.4*expected and 0 < vy < cfg.height*cfg.roi_top_y and .15*cfg.width < vx < .85*cfg.width):
                continue
            if not 2.1 < width < 5.0 or abs(ml-mr) > .22:
                continue
            score = 4*abs(pixel_width/expected-1) + 2*abs(ml-mr) - .0003*min(wl+wr, 700)
            pairs.append((score, left, right))
    if pairs:
        _, left, right = min(pairs, key=lambda p: p[0])
        return left, right
    # A lone curb or marking supplies heading and a metric lane-width prior.
    single = [(weight/(1+8*abs(properties(line)[1])), side, line)
              for side, candidates in enumerate(choices) for line, weight in candidates
              if abs(properties(line)[1]) < .25
              and abs(properties(line)[2] + (cfg.lane_width_m/2 if side == 0 else -cfg.lane_width_m/2)) < cfg.lane_width_m*.6]
    if not single:
        return None, None
    _, side, line = max(single, key=lambda item: item[0])
    return (line, None) if side == 0 else (None, line)


def _ground_point(x: float, y: float, cfg: LaneDetectionConfig) -> tuple[float, float]:
    """Flat road pinhole projection, camera 1.5 m forward, 1.6 m high, pitch -8 deg."""
    f = cfg.width/2  # horizontal FOV 90 degrees
    pitch = math.radians(8)
    v = y-cfg.height/2
    denominator = f*math.sin(pitch)+v*math.cos(pitch)
    return ((x-cfg.width/2)*1.6/denominator,
            1.5+1.6*(f*math.cos(pitch)-v*math.sin(pitch))/denominator)


def detect_lane(bgr: np.ndarray, cfg: LaneDetectionConfig = LaneDetectionConfig(),
                prior_width_px: Optional[float] = None) -> LaneEstimate:
    if bgr.shape != (cfg.height, cfg.width, 3) or bgr.dtype != np.uint8:
        raise ValueError("Expected uint8 BGR image matching configured image size")
    left, right = _fit_sides(_segments(bgr, cfg), cfg, prior_width_px)
    if left is None and right is None:
        return LaneEstimate(None, None, 0.0, None, None)
    bottom, ahead = cfg.height-1, (cfg.height-1)*cfg.lookahead_fraction
    confidence = 1.0 if left is not None and right is not None else 0.5
    width = (right[0]-left[0])*bottom + right[1]-left[1] if confidence == 1 else (
        prior_width_px if prior_width_px is not None else cfg.width*cfg.roi_bottom_width)
    if not math.isfinite(width) or width <= 0:
        return LaneEstimate(None, None, 0.0, left, right)
    if confidence == 1:
        centre = ((left[0]+right[0])/2, (left[1]+right[1])/2)
    else:
        boundary = left if left is not None else right
        centre = (boundary[0], boundary[1] + (width/2 if left is not None else -width/2))
    xb, xa = centre[0]*bottom+centre[1], centre[0]*ahead+centre[1]
    if confidence == 1:
        ground_bottom = _ground_point(xb, bottom, cfg)
        ground_ahead = _ground_point(xa, ahead, cfg)
        slope = (ground_ahead[0]-ground_bottom[0])/(ground_ahead[1]-ground_bottom[1])
        # Scale the observed lane to its metric width, retaining compatibility with
        # arbitrary image sizes and lane-width priors.
        scale = cfg.lane_width_m / (_ground_point(width+cfg.width/2, bottom, cfg)[0])
        offset = -(ground_bottom[0]-slope*ground_bottom[1])*scale
        heading = -math.atan(slope*scale)
    else:
        boundary = left if left is not None else right
        x0, z0 = _ground_point(boundary[0]*bottom+boundary[1], bottom, cfg)
        x1, z1 = _ground_point(boundary[0]*ahead+boundary[1], ahead, cfg)
        slope = (x1-x0)/(z1-z0)
        if prior_width_px is not None:
            scale = cfg.lane_width_m / _ground_point(width+cfg.width/2, bottom, cfg)[0]
            x0, slope = x0*scale, slope*scale
        offset = -(x0-slope*z0 + (cfg.lane_width_m/2 if left is not None else -cfg.lane_width_m/2))
        heading = -math.atan(slope)

    return LaneEstimate(offset, heading, confidence, left, right)


class LaneSteering:
    """Smooth estimates before applying signed PD; decay commands on lost lanes."""
    def __init__(self, cfg: LaneDetectionConfig = LaneDetectionConfig(),
                 kp: float = 0.25, kd: float = 0.9, alpha: float = 0.5) -> None:
        self.cfg, self.kp, self.kd, self.alpha = cfg, kp, kd, alpha
        self.prior_width_px: Optional[float] = None
        self._smoothed: Optional[tuple[float, float]] = None
        self._history = deque(maxlen=5)
        self._widths = deque(maxlen=31)
        self.steering_angle = 0.0

    def update(self, estimate: LaneEstimate) -> float:
        if estimate.confidence == 0:
            self.steering_angle *= 0.9
            return self.steering_angle
        if estimate.left is not None and estimate.right is not None:
            left, right = estimate.left, estimate.right
            self._widths.append((right[0]-left[0])*(self.cfg.height-1)+right[1]-left[1])
            self.prior_width_px = float(np.median(self._widths))
        values = (estimate.lateral_offset_m, estimate.heading_error_rad)
        self._history.append(values)
        if len(self._history) >= 5:
            values = tuple(np.median(self._history, axis=0))
        if self._smoothed is None:
            self._smoothed = values
        else:
            self._smoothed = tuple(self.alpha*v+(1-self.alpha)*old
                                   for v, old in zip(values, self._smoothed))
        offset, heading = self._smoothed
        # Limit heading authority when lateral displacement gives a clear correction.
        heading_term = self.kd*heading
        if abs(offset) > .1 and len(self._history) > 1:
            heading_term = max(-.5*abs(self.kp*offset), min(.5*abs(self.kp*offset), heading_term))
        command = max(-1.0, min(1.0, -(self.kp*offset+heading_term)))
        limit = .15 if len(self._history) >= 5 else 1.0
        self.steering_angle += max(-limit, min(limit, command-self.steering_angle))
        return self.steering_angle


def draw_overlay(bgr: np.ndarray, estimate: LaneEstimate,
                 cfg: LaneDetectionConfig = LaneDetectionConfig()) -> np.ndarray:
    output = bgr.copy()
    cv2.polylines(output, [_roi(cfg)], True, (255, 160, 0), 2)
    lines = [estimate.left, estimate.right]
    if all(line is not None for line in lines):
        lines.append(tuple((l+r)/2 for l, r in zip(*lines)))
    elif estimate.lateral_offset_m is not None:
        # Project the inferred metric centre; a parallel image-space shift would
        # give a false heading when just one boundary is visible.
        ys = (cfg.height-1, int(cfg.height*cfg.roi_top_y))
        xs = []
        for y in ys:
            _, z = _ground_point(cfg.width/2, y, cfg)
            ground_x = -estimate.lateral_offset_m-math.tan(estimate.heading_error_rad)*z
            denominator = cfg.width/2*math.sin(math.radians(8)) + (y-cfg.height/2)*math.cos(math.radians(8))
            xs.append(cfg.width/2 + ground_x*denominator/1.6)
        a = (xs[1]-xs[0])/(ys[1]-ys[0])
        lines.append((a, xs[0]-a*ys[0]))
    for line, colour in zip(lines, [(0, 255, 0), (0, 255, 255), (255, 0, 255)]):
        if line is not None:
            ys = (cfg.height-1, int(cfg.height*cfg.roi_top_y))
            pts = [(int(line[0]*y+line[1]), y) for y in ys]
            cv2.line(output, *pts, colour, 2)
    text = f"offset={estimate.lateral_offset_m} heading={estimate.heading_error_rad} conf={estimate.confidence:.1f}"
    cv2.putText(output, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return output
