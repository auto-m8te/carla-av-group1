import cv2
import numpy as np
import pytest
from perception.lane_detection import LaneDetectionConfig, LaneEstimate, LaneSteering, detect_lane, draw_overlay

CFG = LaneDetectionConfig()


def road(shift=0, sides=(True, True)):
    rng = np.random.default_rng(7)
    image = np.clip(45 + rng.normal(0, 3, (450, 800, 3)), 0, 255).astype(np.uint8)
    if sides[0]:
        cv2.line(image, (160 + shift, 449), (350 + shift, 270), (255, 255, 255), 6)
    if sides[1]:
        cv2.line(image, (640 + shift, 449), (450 + shift, 270), (0, 255, 255), 6)
    return image


def test_centred_lane_and_overlay():
    image = road()
    estimate = detect_lane(image, CFG)
    assert estimate.confidence == 1
    assert abs(estimate.lateral_offset_m) < 0.2
    assert abs(estimate.heading_error_rad) < 0.05
    assert draw_overlay(image, estimate, CFG).shape == image.shape


def test_offset_sign_and_prior():
    estimate = detect_lane(road(-40), CFG)
    controller = LaneSteering()
    assert estimate.lateral_offset_m > 0
    assert controller.update(estimate) < 0
    single = detect_lane(road(-40, (True, False)), CFG, prior_width_px=480)
    assert single.confidence == 0.5
    assert single.lateral_offset_m == pytest.approx(estimate.lateral_offset_m, abs=0.1)
    assert controller.prior_width_px == pytest.approx(480, abs=10)


def test_blank_and_horizontal_clutter_decay():
    image = road(sides=(False, False))
    cv2.line(image, (70, 380), (730, 380), (255, 255, 255), 5)
    estimate = detect_lane(image, CFG)
    assert estimate == LaneEstimate(None, None, 0, None, None)
    controller = LaneSteering()
    previous = controller.update(detect_lane(road(-40), CFG))
    assert controller.update(estimate) == pytest.approx(previous * 0.9)
    assert controller.update(estimate) == pytest.approx(previous * 0.9 ** 2)


def test_pd_smoothing_and_clamping():
    controller = LaneSteering()
    assert controller.update(LaneEstimate(1, 0, 1, None, None)) == -0.25
    assert controller.update(LaneEstimate(0, 0, 1, None, None)) == -0.125
    assert controller.update(LaneEstimate(100, 0, 1, None, None)) == -1

def test_heading_sign():
    image = np.full((450, 800, 3), 45, dtype=np.uint8)
    cv2.line(image, (160, 449), (320, 270), (255, 255, 255), 6)
    cv2.line(image, (640, 449), (420, 270), (0, 255, 255), 6)
    estimate = detect_lane(image, CFG)
    assert estimate.heading_error_rad > 0
    assert LaneSteering().update(estimate) < 0
