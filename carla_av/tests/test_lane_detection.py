import cv2
import numpy as np
import pytest
from perception.lane_detection import LaneDetectionConfig, LaneEstimate, LaneSteering, detect_lane, draw_overlay

CFG = LaneDetectionConfig()


def road(shift=0, sides=(True, True)):
    rng = np.random.default_rng(7)
    image = np.clip(45 + rng.normal(0, 3, (450, 800, 3)), 0, 255).astype(np.uint8)
    # Parallel road boundaries converge at the calibrated camera horizon.
    if sides[0]:
        cv2.line(image, (160 + shift, 449), (round(400 + (160 + shift - 400) * (270-169)/(449-169)), 270), (255, 255, 255), 6)
    if sides[1]:
        cv2.line(image, (640 + shift, 449), (round(400 + (640 + shift - 400) * (270-169)/(449-169)), 270), (0, 255, 255), 6)
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
    cv2.line(image, (160, 449), (293, 270), (255, 255, 255), 6)
    cv2.line(image, (640, 449), (467, 270), (0, 255, 255), 6)
    estimate = detect_lane(image, CFG)
    assert estimate.heading_error_rad > 0
    # This image has both a rightward heading and a leftward metric offset:
    # the command follows their weighted sum, rather than heading alone.
    assert LaneSteering().update(estimate) == pytest.approx(
        -(0.25*estimate.lateral_offset_m + 0.9*estimate.heading_error_rad))


@pytest.mark.parametrize("filename,true_offset", [
    ("Town01_0_00150.png", 0.28236389797122396),
    ("Town03_Opt_0_00042.png", 0.267922045824605),
    ('Town04_0_00090.png', -0.2812859929512115),
])
def test_real_carla_lane_offset(filename, true_offset):
    from pathlib import Path

    image = cv2.imread(str(Path(__file__).parent / "data" / "lane" / filename))
    assert image is not None
    estimate = detect_lane(image)
    assert estimate.confidence > 0
    assert estimate.lateral_offset_m * true_offset > 0
    assert abs(estimate.lateral_offset_m - true_offset) < 0.6


def test_pair_geometry_and_width_prior():
    from perception.lane_detection import _fit_sides

    # Two parallel road pairs, with the outer pair strongly supported.
    segments = np.array([[[100, 449, 292, 270]],
                         [[700, 449, 508, 270]],
                         [[160, 449, 314, 270]],
                         [[640, 449, 486, 270]]])
    left, right = _fit_sides(segments, CFG, 480)
    assert left is not None and right is not None
    assert (right[0]-left[0])*449+right[1]-left[1] == pytest.approx(480, abs=5)
    assert left[0] < 0 < right[0]
    vanishing_y = (right[1]-left[1])/(left[0]-right[0])
    assert 0 < vanishing_y < CFG.height*CFG.roi_top_y


def test_temporal_outlier_and_rate_limit():
    controller = LaneSteering()
    for _ in range(5):
        controller.update(LaneEstimate(.4, 0, 1, None, None))
    before = controller.steering_angle
    after = controller.update(LaneEstimate(-4, .2, 1, None, None))
    assert after < 0  # A single bad estimate cannot reverse the correction.
    assert abs(after-before) <= .15
    for _ in range(5):
        before = controller.steering_angle
        after = controller.update(LaneEstimate(-4, 0, 1, None, None))
        assert abs(after-before) <= .15 + 1e-12


def test_heading_cannot_reverse_established_offset_correction():
    controller = LaneSteering()
    for _ in range(6):
        steer = controller.update(LaneEstimate(.5, -.2, 1, None, None))
    assert steer < 0
