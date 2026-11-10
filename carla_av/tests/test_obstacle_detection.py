"""Unit tests for the LiDAR obstacle detector (E3, Abir) on synthetic point clouds.

Vehicle frame (CARLA): x forward, y right, z up, origin on the ground under the car.
"""

import numpy as np
import pytest

from perception.obstacle_detection import LidarDetectionConfig, detect_obstacle

CFG = LidarDetectionConfig(ego_half_length=2.4, ego_half_width=1.0)


def ground(x_max=60.0, slope=0.0, step=0.5, half_width=8.0):
    """Flat (or sloped) road surface sampled on a grid."""
    xs, ys = np.meshgrid(np.arange(3.0, x_max, step), np.arange(-half_width, half_width, step))
    xs, ys = xs.ravel(), ys.ravel()
    return np.column_stack([xs, ys, slope * xs])


def box(x_rear, y_centre, length=4.0, width=1.8, height=1.5, z0=0.0, step=0.25):
    """Points on the rear face + sides of a parked car (what a LiDAR would hit)."""
    ys = np.arange(y_centre - width / 2, y_centre + width / 2 + 1e-6, step)
    zs = np.arange(z0 + 0.3, z0 + height, step)
    yy, zz = np.meshgrid(ys, zs)
    rear = np.column_stack([np.full(yy.size, x_rear), yy.ravel(), zz.ravel()])
    xs = np.arange(x_rear, x_rear + length, step)
    xx, zz2 = np.meshgrid(xs, zs)
    side = np.column_stack([xx.ravel(), np.full(xx.size, y_centre - width / 2), zz2.ravel()])
    return np.vstack([rear, side])


def test_empty_road_reports_clear():
    det = detect_obstacle(ground(), CFG)
    assert not det.ahead
    assert det.distance is None


def test_no_points_reports_clear():
    det = detect_obstacle(np.empty((0, 3)), CFG)
    assert not det.ahead and det.distance is None


def test_parked_car_in_lane_gives_bumper_gap():
    cloud = np.vstack([ground(), box(x_rear=30.0, y_centre=0.0)])
    det = detect_obstacle(cloud, CFG)
    assert det.ahead
    # gap = rear face of obstacle (30 m) - ego front bumper (2.4 m)
    assert det.distance == pytest.approx(27.6, abs=0.3)


def test_car_in_adjacent_lane_is_ignored():
    cloud = np.vstack([ground(), box(x_rear=20.0, y_centre=-3.5)])
    assert not detect_obstacle(cloud, CFG).ahead


def test_nearest_of_two_obstacles_is_reported():
    cloud = np.vstack([ground(), box(40.0, 0.0), box(15.0, 0.3)])
    assert detect_obstacle(cloud, CFG).distance == pytest.approx(12.6, abs=0.3)


def test_uphill_road_is_not_an_obstacle():
    # 4% grade -> road is 2.4 m above the car's ground plane at 60 m
    assert not detect_obstacle(ground(slope=0.04), CFG).ahead


def test_car_on_uphill_road_still_detected():
    slope = 0.04
    cloud = np.vstack([ground(slope=slope), box(35.0, 0.0, z0=slope * 35.0)])
    det = detect_obstacle(cloud, CFG)
    assert det.ahead and det.distance == pytest.approx(32.6, abs=0.4)


def test_overhead_structure_is_ignored():
    sign = np.column_stack([np.full(40, 25.0), np.linspace(-2, 2, 40), np.full(40, 5.0)])
    assert not detect_obstacle(np.vstack([ground(), sign]), CFG).ahead


def test_sparse_noise_points_are_ignored():
    noise = np.array([[12.0, 0.1, 0.8], [33.0, -0.4, 1.1]])
    assert not detect_obstacle(np.vstack([ground(), noise]), CFG).ahead


def test_beyond_range_is_ignored():
    cloud = np.vstack([ground(x_max=90.0), box(75.0, 0.0)])
    assert not detect_obstacle(cloud, CFG).ahead


def test_curved_corridor_follows_the_turn():
    # Right-hand curve, radius 100 m: lane centre at x=30 is ~4.5 m to the right.
    curvature = 1.0 / 100.0
    y_lane = 0.5 * curvature * 30.0 ** 2
    cloud = np.vstack([ground(), box(30.0, y_lane)])
    assert not detect_obstacle(cloud, CFG).ahead            # straight corridor misses it
    det = detect_obstacle(cloud, CFG, curvature=curvature)  # curved corridor finds it
    assert det.ahead and det.distance == pytest.approx(27.6, abs=0.3)


def test_detection_returns_nearest_point_for_visualisation():
    det = detect_obstacle(np.vstack([ground(), box(20.0, 0.0)]), CFG)
    assert det.nearest_point is not None
    assert det.nearest_point[0] == pytest.approx(20.0, abs=0.3)
    assert det.num_points >= CFG.min_points


def test_invalid_shape_raises():
    with pytest.raises(ValueError):
        detect_obstacle(np.zeros((5, 2)), CFG)


def test_path_polyline_corridor_follows_the_lane():
    # Lane bends right: centreline given as (x, y) points in the vehicle frame.
    xs = np.arange(0.0, 62.0, 2.0)
    path = np.column_stack([xs, 0.5 * xs ** 2 / 100.0])
    y_lane = 0.5 * 30.0 ** 2 / 100.0
    in_lane = np.vstack([ground(), box(30.0, y_lane)])
    det = detect_obstacle(in_lane, CFG, path=path)
    assert det.ahead and det.distance == pytest.approx(27.6, abs=0.3)


def test_path_corridor_ignores_guardrail_beside_curved_lane():
    # A continuous rail 3 m outside the curve: a straight corridor would hit it.
    xs = np.arange(0.0, 62.0, 2.0)
    path = np.column_stack([xs, 0.5 * xs ** 2 / 100.0])
    rx = np.arange(5.0, 60.0, 0.25)
    rail = np.column_stack([np.repeat(rx, 3), np.repeat(0.5 * rx ** 2 / 100.0 - 3.0, 3),
                            np.tile([0.4, 0.6, 0.8], rx.size)])
    cloud = np.vstack([ground(), rail])
    assert not detect_obstacle(cloud, CFG, path=path).ahead


def test_low_terrain_beside_elevated_road_does_not_lift_the_road():
    # Elevated highway (Town04): terrain 8 m below, 10-20 m to the side.
    xs, ys = np.meshgrid(np.arange(3.0, 60.0, 0.5), np.arange(10.0, 20.0, 0.5))
    terrain = np.column_stack([xs.ravel(), ys.ravel(), np.full(xs.size, -8.0)])
    assert not detect_obstacle(np.vstack([ground(half_width=6.0), terrain]), CFG).ahead
