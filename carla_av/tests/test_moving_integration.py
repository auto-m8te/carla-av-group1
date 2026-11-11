"""CARLA glue exercised with local actor doubles; never connects to a server."""

from types import SimpleNamespace

import carla
import numpy as np
import pytest

from avoidance.carla_adapter import CarlaAvoidanceBridge, ground_truth_obstacle_distance
from avoidance.collision_avoidance import AvoidanceConfig
from perception.lidar_sensor import LidarObstacleDetector
from perception.obstacle_detection import LidarDetectionConfig
from perception.tracking import GapTracker


class Actors(list):
    def filter(self, pattern: str) -> list:
        return self


class Vehicle:
    def __init__(self, identity: int, x: float, y: float, velocity: float = 0.0) -> None:
        self.id = identity
        self.transform = carla.Transform(carla.Location(x=x, y=y))
        self.velocity = carla.Vector3D(x=velocity)
        self.bounding_box = carla.BoundingBox(carla.Location(), carla.Vector3D(2.0, 1.0, 1.0))

    def get_transform(self):
        return self.transform

    def get_location(self):
        return self.transform.location

    def get_velocity(self):
        return self.velocity


class Waypoint:
    lane_type = carla.LaneType.Driving
    lane_id = -1

    def __init__(self, x: float = 0.0) -> None:
        self.transform = carla.Transform(carla.Location(x=x))

    def next(self, distance: float) -> list:
        return [Waypoint(self.transform.location.x + distance)]

    def previous(self, distance: float) -> list:
        return [Waypoint(self.transform.location.x - distance)]

    def get_left_lane(self):
        return self


def make_world(actors: list) -> SimpleNamespace:
    snapshot = SimpleNamespace(frame=1, timestamp=SimpleNamespace(delta_seconds=0.05))
    cmap = SimpleNamespace(get_waypoint=lambda loc: Waypoint(loc.x))
    return SimpleNamespace(get_actors=lambda: Actors(actors), get_map=lambda: cmap,
                           get_snapshot=lambda: snapshot)


@pytest.mark.parametrize("x,velocity,expected", [
    (55, -7, False), (55, 8, True), (55, 0, False),
    (8, 8, False), (-10, -7, False), (-20, -7, True),
])
def test_truth_adjacent_velocity_horizon(x, velocity, expected):
    ego, other = Vehicle(1, 0, 0, 8), Vehicle(2, x, -3.5, velocity)
    bridge = CarlaAvoidanceBridge(make_world([ego, other]), ego, AvoidanceConfig())
    assert bridge.ground_truth_adjacent_lane_clear() == expected


def test_truth_moving_run_sees_oncoming_beyond_original_range():
    ego, other = Vehicle(1, 0, 0, 8), Vehicle(2, 110, -3.5, -7)
    cfg = AvoidanceConfig(adjacent_detection_range=180.0)
    bridge = CarlaAvoidanceBridge(make_world([ego, other]), ego, cfg)
    assert not bridge.ground_truth_adjacent_lane_clear()


def test_interpolated_truth_gap_tracks_stationary_actor_without_false_motion():
    ego, parked = Vehicle(1, 0, 0, 8), Vehicle(2, 40.3, 0)
    world = make_world([ego, parked])
    gap = ground_truth_obstacle_distance(world, interpolate=True)
    tracker = GapTracker()
    for _ in range(30):
        tracker.update(gap(ego), 8.0, 0.05)
        ego.transform.location.x += 0.4
    assert tracker.speed == pytest.approx(0.0, abs=0.001)


def test_bridge_tracks_once_per_tick_and_resets_when_lead_disappears():
    ego, other = Vehicle(1, 0, 0, 8), Vehicle(2, 35, 0, 4)
    world = make_world([ego, other])
    bridge = CarlaAvoidanceBridge(world, ego, AvoidanceConfig())
    assert bridge.perceive().obstacle_speed is None
    assert bridge.perceive().obstacle_speed is None
    for _ in range(2):
        ego.transform.location.x += 0.4
        other.transform.location.x += 0.2
        world.get_snapshot().frame += 1
        p = bridge.perceive()
    assert p.obstacle_speed == pytest.approx(4.0, abs=0.001)
    other.transform.location.y = 10.0
    world.get_snapshot().frame += 1
    assert bridge.perceive().obstacle_speed is None


def test_lidar_oncoming_warmup_approach_and_departure(monkeypatch):
    monkeypatch.setattr("perception.lidar_sensor.lane_path",
                        lambda *args, **kwargs: [(-20, -3.5), (65, -3.5)])
    detector = object.__new__(LidarObstacleDetector)
    detector.vehicle = Vehicle(1, 0, 0, 0)
    detector.world = make_world([detector.vehicle])
    detector.config = LidarDetectionConfig(ego_half_length=2.0)
    detector.avoidance_config = AvoidanceConfig()
    detector._adj_tracker, detector._adj_frame = GapTracker(), None
    detector._tags = None
    detector._refresh = lambda: False
    xs, ys = np.meshgrid(np.arange(-15, 60, 0.5), np.arange(-6, 2, 0.5))
    road = np.column_stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)])
    for x in (50.0, 49.65, 49.3, 8.0, -8.0):
        car = np.array([[x, -3.5, 0.6], [x, -3.4, 1.0], [x, -3.6, 1.4]])
        detector._points = np.vstack([road, car])
        detector.world.get_snapshot().frame += 1
        assert not detector.adjacent_lane_clear(Waypoint(), -3.5, 3.5)
    detector._points = road
    detector.world.get_snapshot().frame += 1
    assert detector.adjacent_lane_clear(Waypoint(), -3.5, 3.5)


@pytest.mark.parametrize("plugin", [False, True])
@pytest.mark.parametrize("dt", [0.05, 0.1])
def test_bridge_debounces_both_sources_once_per_tick(plugin, dt):
    ego = Vehicle(1, 0, 0)
    world = make_world([ego])
    world.get_snapshot().timestamp.delta_seconds = dt
    raw = [True]
    bridge = CarlaAvoidanceBridge(world, ego, AvoidanceConfig(),
        adjacent_clear_fn=(lambda *args: raw[0]) if plugin else None)
    bridge.ground_truth_adjacent_lane_clear = lambda: raw[0]
    ticks = round(AvoidanceConfig().adjacent_clear_hold / dt)
    for _ in range(ticks - 1):
        assert not bridge.adjacent_lane_clear()
        assert not bridge.adjacent_lane_clear()
        assert bridge.ground_truth_adjacent_lane_clear()
        world.get_snapshot().frame += 1
    assert bridge.adjacent_lane_clear()
    raw[0] = False
    world.get_snapshot().frame += 1
    assert not bridge.adjacent_lane_clear()
    raw[0] = True
    world.get_snapshot().frame += 1
    assert not bridge.adjacent_lane_clear()


def test_bridge_debounce_falls_back_without_snapshot():
    ego = Vehicle(1, 0, 0)
    world = make_world([ego])
    del world.get_snapshot
    bridge = CarlaAvoidanceBridge(world, ego, AvoidanceConfig(),
                                  adjacent_clear_fn=lambda *args: True)
    for _ in range(19):
        assert not bridge.adjacent_lane_clear()
    assert bridge.adjacent_lane_clear()


@pytest.mark.parametrize("lane_id,behind", [(1, 0.0), (-2, 15.0)])
def test_bridge_passes_direction_specific_lookbehind(lane_id, behind):
    ego = Vehicle(1, 0, 0)
    bridge = CarlaAvoidanceBridge(make_world([ego]), ego, AvoidanceConfig())
    adjacent = Waypoint()
    adjacent.lane_id = lane_id
    bridge._adjacent_wp = lambda home: adjacent
    calls = []
    bridge.adjacent_clear_fn = lambda *args: calls.append(args) or True
    bridge.adjacent_lane_clear()
    assert calls == [(bridge._home_wp, -3.5, 3.5, behind)]


def test_bridge_accepts_legacy_adjacent_plugin():
    ego = Vehicle(1, 0, 0)
    calls = []
    bridge = CarlaAvoidanceBridge(make_world([ego]), ego, AvoidanceConfig(),
        adjacent_clear_fn=lambda home, offset, width: calls.append((offset, width)) or True)
    bridge.adjacent_lane_clear()
    assert calls == [(-3.5, 3.5)]


@pytest.mark.parametrize("lane_id,expected", [(1, False), (-2, True)])
def test_cruise_reanchors_only_same_direction_lane(lane_id, expected):
    from avoidance.collision_avoidance import ControlCommand, Mode
    ego = Vehicle(1, 0, -3.5)
    ego.apply_control = lambda control: None
    world = make_world([ego])
    under_car = Waypoint()
    under_car.lane_id = lane_id
    world.get_map().get_waypoint = lambda loc: under_car
    bridge = CarlaAvoidanceBridge(world, ego, AvoidanceConfig())
    home = bridge._home_wp = Waypoint()
    bridge.apply(ControlCommand(0.0, 0.0, 0.0, Mode.CRUISE))
    assert (bridge._home_wp is under_car) == expected
    if not expected:
        assert bridge._home_wp is home


@pytest.mark.parametrize("x,expected", [(-8.0, True), (-1.5, False), (1.5, False)])
def test_oncoming_truth_ignores_departed_car_but_checks_beside(x, expected):
    ego, other = Vehicle(1, 0, 0), Vehicle(2, x, -3.5, -7)
    bridge = CarlaAvoidanceBridge(make_world([ego, other]), ego, AvoidanceConfig())
    adjacent = Waypoint()
    adjacent.lane_id = 1
    bridge._adjacent_wp = lambda home: adjacent
    assert bridge.ground_truth_adjacent_lane_clear() == expected


@pytest.mark.parametrize("x,expected", [(-8.0, True), (-1.5, False), (1.5, False)])
def test_lidar_zero_lookbehind_keeps_bumper_band(monkeypatch, x, expected):
    path_calls = []
    def path(*args, **kwargs):
        path_calls.append(kwargs)
        return [(0, -3.5), (65, -3.5)]
    monkeypatch.setattr("perception.lidar_sensor.lane_path", path)
    detector = object.__new__(LidarObstacleDetector)
    detector.vehicle = Vehicle(1, 0, 0)
    detector.world = make_world([detector.vehicle])
    detector.config = LidarDetectionConfig(ego_half_length=2.0)
    detector.avoidance_config = AvoidanceConfig()
    detector._adj_tracker, detector._adj_frame = GapTracker(), None
    detector._tags = None
    detector._refresh = lambda: False
    xs, ys = np.meshgrid(np.arange(-15, 60, 0.5), np.arange(-6, 2, 0.5))
    road = np.column_stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)])
    car = np.array([[x, -3.5, 0.6], [x, -3.4, 1.0], [x, -3.6, 1.4]])
    detector._points = np.vstack([road, car])
    assert detector.adjacent_lane_clear(Waypoint(), -3.5, 3.5, 0.0) == expected
    assert path_calls[0]["behind"] == 0.0


def test_explicit_home_waypoint_avoids_spawn_lane_lookup():
    ego = Vehicle(1, 0, 0)
    world = make_world([ego])
    home, wrong = Waypoint(), Waypoint()
    wrong.transform.location.y = 7.0
    calls = []
    world.get_map().get_waypoint = lambda loc: calls.append(loc) or wrong
    bridge = CarlaAvoidanceBridge(world, ego, AvoidanceConfig(), home_waypoint=home)
    assert bridge._home_waypoint() is home
    assert bridge.lateral_offset() == 0.0
    assert calls == []
