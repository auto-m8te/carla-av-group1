"""Colour/edge lane detection and signed PD lane keeping, independent of CARLA."""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass
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
    roi_top_y: float = 0.6
    roi_top_width: float = 0.15
    roi_bottom_width: float = 0.95
    white_s_max: int = 39
    white_v_min: int = 181
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
    white = cv2.inRange(hsv, (0, 0, cfg.white_v_min), (179, cfg.white_s_max, 255))
    yellow = cv2.inRange(hsv, (cfg.yellow_h_min, cfg.yellow_s_min, cfg.yellow_v_min),
                         (cfg.yellow_h_max, 255, 255))
    edges = cv2.Canny(white | yellow, cfg.canny_low, cfg.canny_high)
    mask = np.zeros(edges.shape, dtype=np.uint8)
    cv2.fillPoly(mask, [_roi(cfg)], 255)
    return cv2.HoughLinesP(edges & mask, cfg.hough_rho, cfg.hough_theta,
                           cfg.hough_threshold, minLineLength=cfg.min_line_length,
                           maxLineGap=cfg.max_line_gap)


def _fit_sides(segments: Optional[np.ndarray], cfg: LaneDetectionConfig
               ) -> tuple[Optional[Line], Optional[Line]]:
    sides: list[list[tuple[float, float, float]]] = [[], []]
    if segments is not None:
        for x1, y1, x2, y2 in segments.reshape(-1, 4):
            dx, dy = float(x2-x1), float(y2-y1)
            if dy == 0 or (dx != 0 and abs(dy/dx) < cfg.min_slope):
                continue
            a = dx/dy
            bottom = x1 + a*(cfg.height-1-y1)
            side = 0 if a < 0 and bottom < cfg.width/2 else 1
            if (side == 1 and (a <= 0 or bottom <= cfg.width/2)):
                continue
            weight = math.hypot(dx, dy)
            sides[side].extend([(y1, x1, weight), (y2, x2, weight)])
    fitted: list[Optional[Line]] = []
    for points in sides:
        if not points:
            fitted.append(None)
            continue
        y, x, weights = np.array(points).T
        design = np.column_stack((y, np.ones_like(y)))
        a, b = np.linalg.lstsq(design * np.sqrt(weights[:, None]),
                              x * np.sqrt(weights), rcond=None)[0]
        fitted.append((float(a), float(b)))
    return fitted[0], fitted[1]


def detect_lane(bgr: np.ndarray, cfg: LaneDetectionConfig = LaneDetectionConfig(),
                prior_width_px: Optional[float] = None) -> LaneEstimate:
    if bgr.shape != (cfg.height, cfg.width, 3) or bgr.dtype != np.uint8:
        raise ValueError("Expected uint8 BGR image matching configured image size")
    left, right = _fit_sides(_segments(bgr, cfg), cfg)
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
    offset = (cfg.width/2-xb)*cfg.lane_width_m/width
    heading = math.atan2(xb-xa, bottom-ahead)
    return LaneEstimate(offset, heading, confidence, left, right)


class LaneSteering:
    """Smooth estimates before applying signed PD; decay commands on lost lanes."""
    def __init__(self, cfg: LaneDetectionConfig = LaneDetectionConfig(),
                 kp: float = 0.25, kd: float = 0.9, alpha: float = 0.5) -> None:
        self.cfg, self.kp, self.kd, self.alpha = cfg, kp, kd, alpha
        self.prior_width_px: Optional[float] = None
        self._smoothed: Optional[tuple[float, float]] = None
        self.steering_angle = 0.0

    def update(self, estimate: LaneEstimate) -> float:
        if estimate.confidence == 0:
            self.steering_angle *= 0.9
            return self.steering_angle
        if estimate.left is not None and estimate.right is not None:
            left, right = estimate.left, estimate.right
            self.prior_width_px = (right[0]-left[0])*(self.cfg.height-1)+right[1]-left[1]
        values = (estimate.lateral_offset_m, estimate.heading_error_rad)
        if self._smoothed is None:
            self._smoothed = values
        else:
            self._smoothed = tuple(self.alpha*v+(1-self.alpha)*old
                                   for v, old in zip(values, self._smoothed))
        offset, heading = self._smoothed
        self.steering_angle = max(-1.0, min(1.0, -(self.kp*offset+self.kd*heading)))
        return self.steering_angle


def draw_overlay(bgr: np.ndarray, estimate: LaneEstimate,
                 cfg: LaneDetectionConfig = LaneDetectionConfig()) -> np.ndarray:
    output = bgr.copy()
    cv2.polylines(output, [_roi(cfg)], True, (255, 160, 0), 2)
    lines = [estimate.left, estimate.right]
    if all(line is not None for line in lines):
        lines.append(tuple((l+r)/2 for l, r in zip(*lines)))
    elif estimate.lateral_offset_m is not None:
        # Recover the inferred centre using the same width and heading as detection.
        boundary = estimate.left if estimate.left is not None else estimate.right
        xb = boundary[0]*(cfg.height-1)+boundary[1]
        sign = 1 if estimate.left is not None else -1
        denominator = sign/2 + estimate.lateral_offset_m/cfg.lane_width_m
        if abs(denominator) > 1e-9:
            width = (cfg.width/2-xb)/denominator
            lines.append((boundary[0], boundary[1]+sign*width/2))
    for line, colour in zip(lines, [(0, 255, 0), (0, 255, 255), (255, 0, 255)]):
        if line is not None:
            ys = (cfg.height-1, int(cfg.height*cfg.roi_top_y))
            pts = [(int(line[0]*y+line[1]), y) for y in ys]
            cv2.line(output, *pts, colour, 2)
    text = f"offset={estimate.lateral_offset_m} heading={estimate.heading_error_rad} conf={estimate.confidence:.1f}"
    cv2.putText(output, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return output
