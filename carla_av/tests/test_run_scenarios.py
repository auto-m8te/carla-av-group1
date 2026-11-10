"""Curve geometry tests using waypoints only; no CARLA server needed."""

from types import SimpleNamespace

import carla
import pytest

from scenarios.run_scenarios import curve_is_usable, curve_obstacle_distance, is_finished


class FakeWaypoint:
    def __init__(self, yaws: list[float], index: int = 0, bad: int = -1,
                 adjacent: bool = True) -> None:
        self.yaws, self.index, self.bad, self.adjacent = yaws, index, bad, adjacent
        self.transform = SimpleNamespace(rotation=SimpleNamespace(yaw=yaws[index]))
        self.is_junction = index == bad
        self.lane_type = carla.LaneType.Driving

    def next(self, distance: float) -> list:
        index = self.index + int(distance / 5)
        return ([FakeWaypoint(self.yaws, index, self.bad, self.adjacent)]
                if index < len(self.yaws) else [])

    def get_left_lane(self):
        return SimpleNamespace(lane_type=carla.LaneType.Driving) if self.adjacent else None

    def get_right_lane(self):
        return self.get_left_lane()


@pytest.mark.parametrize("side", [-1, 1])
@pytest.mark.parametrize("turn,usable", [(0, False), (44, False), (45, True),
                                          (80, True), (120, True), (121, False)])
def test_curve_total_heading(side, turn, usable):
    start = FakeWaypoint([170 + turn * min(i, 16) / 16 for i in range(21)])
    assert curve_is_usable(start, 100.0, side) == usable


@pytest.mark.parametrize("bad,adjacent", [(10, True), (20, True), (-1, False)])
def test_curve_requires_whole_stretch(bad, adjacent):
    start = FakeWaypoint([i * 5 for i in range(21)], bad=bad, adjacent=adjacent)
    assert not curve_is_usable(start, 100.0, -1)


def test_curve_rejects_kink_and_missing_road():
    assert not curve_is_usable(FakeWaypoint([0] + [60] * 20), 100.0, -1)
    assert not curve_is_usable(FakeWaypoint([i * 5 for i in range(16)]), 100.0, -1)


@pytest.mark.parametrize("yaws,expected", [
    ([i * 5 for i in range(17)], 40.0),
    ([min(i, 4) * 20 for i in range(17)], 25.0),
    ([max(0, i - 12) * 20 for i in range(17)], 55.0),
])
def test_curve_apex_distance(yaws, expected):
    assert curve_obstacle_distance(FakeWaypoint(yaws)) == pytest.approx(expected)


def test_curve_finish_rules():
    row = {"collision": False, "odometer": 99.0}
    assert not is_finished("curve_lane", row, 0.0, 0.0)
    row["odometer"] = 100.0
    assert is_finished("curve_lane", row, 0.0, 0.0)
    assert not is_finished("curve_stop", row, 2.9, 0.0)
    assert is_finished("curve_stop", row, 3.0, 0.0)


def test_curve_accumulates_heading_and_handles_yaw_wrap():
    yaws = [170 + 5 * min(i, 8) if i <= 8 else 210 - 5 * (i - 8)
            for i in range(21)]
    wrapped = [(yaw + 180) % 360 - 180 for yaw in yaws]
    assert curve_is_usable(FakeWaypoint(wrapped), 100.0, -1)


def test_moving_finish_rules():
    row = {"collision": False, "odometer": 119.9, "mode": "FOLLOW"}
    assert not is_finished("lead_vehicle", row, 0.0, 0.0)
    row["odometer"] = 120.0
    assert is_finished("lead_vehicle", row, 0.0, 0.0)
    assert not is_finished("lead_brake", row, 2.9, 0.0)
    assert is_finished("lead_brake", row, 3.0, 0.0)
    assert not is_finished("oncoming_overtake", row, 0.0, 30.0)
    row["mode"] = "CRUISE"
    assert is_finished("oncoming_overtake", row, 0.0, 30.0)


def test_scripted_speed_controller():
    from scenarios.run_scenarios import scripted_speed_control

    assert scripted_speed_control(0.0, 4.0) == (0.75, 0.0)
    assert scripted_speed_control(5.0, 4.0) == (0.0, 0.4)
    assert scripted_speed_control(0.0, 0.0) == (0.0, 1.0)
    assert scripted_speed_control(0.5, 0.0) == (0.0, 0.2)
    assert scripted_speed_control(0.3, 0.0) == (0.0, 0.12)
    assert scripted_speed_control(0.29, 0.0) == (0.0, 1.0)


@pytest.mark.parametrize("lead_start,length,expected", [
    (30.0, 140.0, 78.0), (40.0, 140.0, 88.0),
    (35.0, 80.0, 80.0), (35.0, 83.0, 83.0),
])
def test_lead_brake_blocker_distance(lead_start, length, expected):
    from scenarios.run_scenarios import lead_brake_blocker_distance

    assert lead_brake_blocker_distance(lead_start, length) == expected


def test_scripted_deceleration_profile():
    from scenarios.run_scenarios import scripted_deceleration_target

    previous = 6.0
    for _ in range(40):
        target = scripted_deceleration_target(previous, 0.05)
        assert target == pytest.approx(max(0.0, previous - 0.15))
        previous = target
    assert previous == pytest.approx(0.0, abs=1e-12)
    assert scripted_deceleration_target(0.1, 0.1) == 0.0
    assert scripted_deceleration_target(0.0, 0.05) == 0.0
    assert scripted_deceleration_target(6.0, 0.0) == 6.0
    assert scripted_deceleration_target(6.0, 0.2) == pytest.approx(5.4)


def test_lead_brake_road_covers_blocker_and_margin():
    from scenarios.run_scenarios import RunParams, scenario_road_length

    for lead_start in (30.0, 40.0, 120.0):
        params = RunParams("Town01", "lead_brake", 1, lead_start, 8.0)
        assert scenario_road_length(params) >= lead_start + 40.0 + 8.0 + 20.0


def test_scripted_driver_tracks_ramp_each_tick(monkeypatch):
    import scenarios.run_scenarios as runner

    targets = []
    actor = SimpleNamespace(get_location=lambda: SimpleNamespace(distance=lambda other: 0.0),
                            apply_control=lambda control: None)
    monkeypatch.setattr(runner, "waypoint_lane_steer", lambda world: lambda actor: 0.0)
    monkeypatch.setattr(runner, "ground_truth_obstacle_distance", lambda world: lambda actor: None)
    monkeypatch.setattr(runner, "speed_of", lambda actor: 6.0)
    monkeypatch.setattr(runner, "scripted_speed_control",
                        lambda speed, target: (targets.append(target) or 0.0, 0.0))
    driver = runner.ScriptedDriver(None, actor, "lead_brake", 6.0)
    driver.travelled = 39.9
    driver.step()
    driver.travelled = 40.0
    driver.step()
    driver.step()
    assert targets == pytest.approx([6.0, 5.85, 5.70])


@pytest.mark.parametrize("scenario,distance,expected", [
    ("lead_vehicle", 50.0, 4.0), ("lead_brake", 39.9, 4.0),
    ("lead_brake", 40.0, 0.0), ("oncoming_overtake", 50.0, 4.0),
])
def test_scripted_target_speed(scenario, distance, expected):
    from scenarios.run_scenarios import scripted_target_speed

    assert scripted_target_speed(scenario, 4.0, distance) == expected


@pytest.mark.parametrize("scenario,moving,blocker", [
    ("lead_vehicle", [True], False), ("lead_brake", [True, False], True),
    ("oncoming_overtake", [False, True], False),
])
def test_scripted_actor_placement(scenario, moving, blocker):
    import random
    from avoidance.collision_avoidance import AvoidanceConfig
    from scenarios.run_scenarios import RunParams, _scenario_vehicles

    class RoadPoint:
        def __init__(self, x=0.0, lane_id=-1):
            self.x, self.lane_id = x, lane_id
            self.lane_type = carla.LaneType.Driving

        def next(self, distance):
            return [RoadPoint(self.x + distance, self.lane_id)]

        def get_left_lane(self):
            return RoadPoint(self.x, -self.lane_id)

    params = RunParams("Town01", scenario, 1, 35.0, 8.0)
    vehicles = _scenario_vehicles(RoadPoint(), params, AvoidanceConfig(), random.Random(42))
    assert [entry[2] for entry in vehicles] == moving
    assert vehicles[0][1].x == 35.0
    if blocker:
        assert vehicles[1][1].x == params.obstacle_m + 40.0 + 8.0
    if scenario == "oncoming_overtake":
        assert 75.0 <= vehicles[1][1].x <= 95.0
        assert vehicles[1][1].lane_id > 0


@pytest.mark.parametrize("scenario", ["empty_lane", "curve_stop", "lead_vehicle",
                                      "lead_brake", "oncoming_overtake"])
def test_scenario_configuration_preserves_legacy_settings(scenario):
    from avoidance.collision_avoidance import AvoidanceConfig
    from scenarios.run_scenarios import RunParams, scenario_config, MOVING_SCENARIOS

    base = AvoidanceConfig()
    params = RunParams("Town01", scenario, 1, 35.0, 7.0)
    cfg = scenario_config(base, params, 3.7)
    assert cfg.target_speed == 7.0 and cfg.lane_width == 3.7
    if scenario in MOVING_SCENARIOS:
        assert cfg.safety_margin == (10.0 if scenario == "oncoming_overtake" else 6.0)
        assert cfg.adjacent_detection_range == 180.0
    else:
        assert cfg.safety_margin == base.safety_margin
        assert cfg.adjacent_detection_range == base.adjacent_detection_range


@pytest.mark.parametrize("scenario", ["moving", "everything", "lead_vehicle",
                                      "lead_brake", "oncoming_overtake"])
def test_cli_accepts_moving_scenarios(monkeypatch, scenario):
    from scenarios.run_scenarios import _parse_args

    monkeypatch.setattr("sys.argv", ["run_scenarios", "--scenario", scenario])
    assert _parse_args().scenario == scenario

def test_camera_lane_flags_and_columns(monkeypatch):
    from scenarios.run_scenarios import _parse_args, CSV_FIELDS
    monkeypatch.setattr('sys.argv', ['runner', '--lane', 'camera', '--lane-debug', 'overlays'])
    args = _parse_args()
    assert args.lane == 'camera'
    assert args.lane_debug == 'overlays'
    assert CSV_FIELDS[-3:] == ('lane_conf', 'lane_offset_cam', 'gt_lane_steer')

def test_camera_log_values():
    from scenarios.run_scenarios import lane_log_values
    from perception.lane_detection import LaneEstimate
    camera = SimpleNamespace(last_estimate=LaneEstimate(None, None, 0, None, None))
    assert lane_log_values(camera, 0.2) == dict(lane_conf=0, lane_offset_cam='', gt_lane_steer=0.2)
    assert lane_log_values(None, -0.1) == dict(lane_conf='', lane_offset_cam='', gt_lane_steer=-0.1)


def test_lidar_noise_reaches_the_sensor_mount():
    from avoidance.collision_avoidance import AvoidanceConfig
    from scenarios.run_scenarios import RunParams, lidar_mount

    static = RunParams("Town04", "obstacle_stop", 1, 40.0, 8.0, lidar_noise=0.05)
    moving = RunParams("Town01", "oncoming_overtake", 1, 40.0, 8.0, lidar_noise=0.05)
    assert lidar_mount(static, AvoidanceConfig()).noise_stddev == 0.05
    assert lidar_mount(moving, AvoidanceConfig()).noise_stddev == 0.05
    assert lidar_mount(RunParams("Town04", "empty_lane", 1, 40.0, 8.0), AvoidanceConfig()).noise_stddev == 0.0
