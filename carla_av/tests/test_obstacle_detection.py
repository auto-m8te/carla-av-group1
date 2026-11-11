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


# ---- adjacent-lane occupancy (is the lane we would swerve into free?) ----

from perception.obstacle_detection import lane_occupied, lane_occupied_points  # noqa: E402

LEFT_LANE = np.array([[-20.0, -3.5], [65.0, -3.5]])   # centreline of the lane to our left


def test_empty_adjacent_lane_is_free():
    assert not lane_occupied(ground(), CFG, LEFT_LANE, lane_width=3.5)


def test_parked_car_in_adjacent_lane_blocks_it():
    cloud = np.vstack([ground(), box(40.0, -3.5)])
    assert lane_occupied(cloud, CFG, LEFT_LANE, lane_width=3.5)


def test_car_behind_in_adjacent_lane_blocks_it():
    cloud = np.vstack([ground(), box(-12.0, -3.5)])
    assert lane_occupied(cloud, CFG, LEFT_LANE, lane_width=3.5, behind=15.0)


def test_obstacle_in_our_own_lane_does_not_block_adjacent_lane():
    cloud = np.vstack([ground(), box(30.0, 0.0)])
    assert not lane_occupied(cloud, CFG, LEFT_LANE, lane_width=3.5)


def test_beyond_lookahead_does_not_block():
    cloud = np.vstack([ground(x_max=90.0), box(70.0, -3.5)])
    assert not lane_occupied(cloud, CFG, LEFT_LANE, lane_width=3.5, ahead=60.0)


def test_own_car_body_is_never_counted():
    # Halfway through a lane change the target lane centre is 2 m to our left, so our
    # own body (|y| <= 1 m) sits inside the target lane band.
    halfway = np.array([[-20.0, -2.0], [65.0, -2.0]])
    body = np.column_stack([np.linspace(-2.0, 2.0, 30), np.full(30, -0.9), np.full(30, 1.2)])
    cloud = np.vstack([ground(), body])
    assert not lane_occupied(cloud, CFG, halfway, lane_width=3.5)


def test_empty_lane_occupied_points_mask() -> None:
    cloud = ground()
    mask = lane_occupied_points(cloud, CFG, LEFT_LANE, lane_width=3.5)
    assert mask.dtype == np.bool_
    assert mask.shape == (len(cloud),)
    assert not mask.any()


def test_parked_car_lane_occupied_points_excludes_ground() -> None:
    road = ground()
    car = box(40.0, -3.5)
    cloud = np.vstack([road, car])
    mask = lane_occupied_points(cloud, CFG, LEFT_LANE, lane_width=3.5)
    assert mask.shape == (len(cloud),)
    assert not mask[:len(road)].any()
    assert mask[len(road):].any()
    assert not mask[len(road):].all()


def test_nothing_beyond_the_end_of_the_path_is_an_obstacle():
    # Road turns at 30 m: the path stops there. A tree straight ahead at 45 m
    # (where the road no longer goes) must not count.
    path = np.array([[0.0, 0.0], [15.0, 0.0], [30.0, 0.0]])
    tree = box(45.0, 0.0, length=1.0, width=1.0, height=2.0)
    assert not detect_obstacle(np.vstack([ground(), tree]), CFG, path=path).ahead


def test_adjacent_check_stops_at_the_end_of_the_lane_path():
    short_left = np.array([[-20.0, -3.5], [30.0, -3.5]])
    building = box(45.0, -3.5, length=5.0, width=3.0, height=2.4)
    assert not lane_occupied(np.vstack([ground(), building]), CFG, short_left, lane_width=3.5)


def test_nearest_adjacent_x_is_ahead_of_front_bumper():
    from perception.obstacle_detection import nearest_occupied_ahead

    cloud = np.vstack([ground(), box(-12.0, -3.5), box(30.0, -3.5), box(45.0, -3.5)])
    assert nearest_occupied_ahead(cloud, CFG, LEFT_LANE, 3.5) == pytest.approx(30.0)
    assert nearest_occupied_ahead(ground(), CFG, LEFT_LANE, 3.5) is None


def test_adjacent_long_range_can_see_scripted_oncoming_before_pullout():
    from perception.obstacle_detection import nearest_occupied_ahead

    path = np.array([[-20.0, -3.5], [180.0, -3.5]])
    cloud = np.vstack([ground(x_max=180), box(110.0, -3.5)])
    assert nearest_occupied_ahead(cloud, CFG, path, 3.5, ahead=180.0) == pytest.approx(110.0)


@pytest.mark.parametrize("adjacent", [False, True])
def test_sparse_road_returns_do_not_hide_distant_car(adjacent):
    from perception.obstacle_detection import nearest_occupied_ahead

    cfg = LidarDetectionConfig(max_range=80.0)
    y = -3.5 if adjacent else 0.0
    xs, zs = np.meshgrid(np.arange(64.0, 65.1, 0.25), np.linspace(0.35, 1.5, 8))
    car = np.column_stack([xs.ravel(), np.full(xs.size, y), zs.ravel()])
    cloud = np.vstack([ground(x_max=45.0), car])
    path = np.array([[-20.0, y], [80.0, y]])
    if adjacent:
        assert nearest_occupied_ahead(cloud, cfg, path, 3.5, ahead=80.0) == pytest.approx(64.0)
    else:
        det = detect_obstacle(cloud, cfg, path=path)
        assert det.ahead
        assert det.distance == pytest.approx(64.0 - cfg.ego_half_length)


def test_flat_raised_road_patch_is_not_an_obstacle():
    # Banked road at a curve (CARLA Town01): a flat patch 0.5 m above the estimated
    # road. It has no vertical extent, unlike a car's rear face.
    xs, ys = np.meshgrid(np.arange(20.0, 24.0, 0.25), np.arange(-1.0, 1.0, 0.25))
    slab = np.column_stack([xs.ravel(), ys.ravel(), np.full(xs.size, 0.5)])
    assert not detect_obstacle(np.vstack([ground(x_max=19.0), slab]), CFG).ahead


def test_car_rear_face_still_detected_with_vertical_check():
    det = detect_obstacle(np.vstack([ground(), box(22.0, 0.0)]), CFG)
    assert det.ahead and det.distance == pytest.approx(19.6, abs=0.3)
