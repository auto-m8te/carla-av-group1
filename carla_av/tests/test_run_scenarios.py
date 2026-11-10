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
        assert 50.0 <= vehicles[1][1].x <= 60.0
        assert vehicles[1][1].x == params.obstacle_m + 20.0
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
        assert cfg.safety_margin == 6.0 and cfg.adjacent_detection_range == 180.0
    else:
        assert cfg.safety_margin == base.safety_margin
        assert cfg.adjacent_detection_range == base.adjacent_detection_range


@pytest.mark.parametrize("scenario", ["moving", "everything", "lead_vehicle",
                                      "lead_brake", "oncoming_overtake"])
def test_cli_accepts_moving_scenarios(monkeypatch, scenario):
    from scenarios.run_scenarios import _parse_args

    monkeypatch.setattr("sys.argv", ["run_scenarios", "--scenario", scenario])
    assert _parse_args().scenario == scenario
