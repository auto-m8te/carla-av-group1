"""Feeds toy-sim traces (same CSV format as CARLA runs) through the analysis."""

import csv

import pytest

from analysis.analyze_results import analyze
from avoidance.collision_avoidance import AvoidanceConfig, CollisionAvoidance, Perception

FIELDS = ("t", "x", "y", "speed", "mode", "obstacle_distance", "lateral_offset",
          "adjacent_lane_clear", "throttle", "steer", "brake", "odometer", "collision")


def write_toy_run(path, obstacle_x, adjacent_clear, steps=900):
    cfg = AvoidanceConfig()
    ctrl = CollisionAvoidance(cfg)
    x = y = 0.0
    v, dt = 8.0, 0.05
    with path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        for i in range(steps):
            in_path = obstacle_x is not None and abs(y) < cfg.lane_width / 2 and obstacle_x > x
            dist = obstacle_x - x if in_path else None
            cmd = ctrl.step(Perception(0.0, dist, v, adjacent_clear, y, x))
            w.writerow(dict(t=i * dt, x=x, y=0, speed=v, mode=cmd.mode.value,
                            obstacle_distance="" if dist is None else dist, lateral_offset=y,
                            adjacent_lane_clear=int(adjacent_clear), throttle=cmd.throttle,
                            steer=cmd.steer, brake=cmd.brake, odometer=x, collision=0))
            v = max(0.0, v + (cmd.throttle * 3.0 - cmd.brake * cfg.max_decel) * dt)
            x += v * dt
            y += cmd.steer * 6.0 * dt


def test_analysis_marks_all_three_toy_scenarios_successful(tmp_path):
    write_toy_run(tmp_path / "empty_lane_run1.csv", None, False)
    write_toy_run(tmp_path / "obstacle_stop_run1.csv", 60.0, False)
    write_toy_run(tmp_path / "obstacle_avoid_run1.csv", 60.0, True)

    metrics = {m.scenario: m for m in analyze(tmp_path, make_plots=True)}

    assert all(m.success for m in metrics.values())
    assert metrics["obstacle_stop"].min_gap_m >= 2.0
    assert metrics["obstacle_avoid"].reaction_distance_m is not None
    assert (tmp_path / "summary.md").read_text().count("|") > 20
    assert len(list((tmp_path / "plots").glob("*.png"))) == 3


def test_collision_marks_run_failed(tmp_path):
    path = tmp_path / "obstacle_stop_run1.csv"
    write_toy_run(path, 60.0, False)
    text = path.read_text().splitlines()
    text[-1] = text[-1][:-1] + "1"  # flip last row's collision flag
    path.write_text("\n".join(text) + "\n")
    (m,) = analyze(tmp_path, make_plots=False)
    assert m.collision and not m.success


def test_perception_error_against_ground_truth():
    from analysis.analyze_results import perception_error

    rows = [
        {"obstacle_distance": 20.5, "gt_obstacle_distance": 20.0},  # error 0.5
        {"obstacle_distance": 10.0, "gt_obstacle_distance": 11.5},  # error 1.5
        {"obstacle_distance": None, "gt_obstacle_distance": 9.0},   # missed
        {"obstacle_distance": 7.0, "gt_obstacle_distance": None},   # false alarm
        {"obstacle_distance": None, "gt_obstacle_distance": None},
    ]
    mae, missed, false = perception_error(rows)
    assert mae == 1.0 and missed == 1 and false == 1


def test_old_csvs_without_ground_truth_column_still_load(tmp_path):
    write_toy_run(tmp_path / "empty_lane_run1.csv", None, False)  # FIELDS has no gt column
    (m,) = analyze(tmp_path, make_plots=False)
    assert m.perception_mae_m is None and m.perception_missed == 0


def test_adjacent_lane_disagreement_is_counted(tmp_path):
    path = tmp_path / "obstacle_avoid_run1.csv"
    write_toy_run(path, 60.0, True)
    lines = path.read_text().splitlines()
    header = lines[0] + ",gt_obstacle_distance,gt_adjacent_lane_clear"
    body = [ln + "," + ("," + ("0" if i < 5 else "1")) for i, ln in enumerate(lines[1:])]
    path.write_text("\n".join([header] + body) + "\n")
    (m,) = analyze(tmp_path, make_plots=False)
    assert m.adjacent_disagree == 5   # toy run logs adjacent_lane_clear=1 everywhere


@pytest.mark.parametrize("scenario,obstacle", [("curve_lane", None), ("curve_stop", 60.0)])
@pytest.mark.parametrize("failure", [None, "collision", "lane", "moving", "gap"])
def test_curve_success_rules(tmp_path, scenario, obstacle, failure):
    path = tmp_path / f"{scenario}_run1.csv"
    write_toy_run(path, obstacle, False)
    with path.open(newline="") as fh:
        rows = list(csv.DictReader(fh))
    if failure == "collision":
        rows[-1]["collision"] = "1"
    elif failure == "lane":
        rows[0]["lateral_offset"] = "1.0"
    elif failure == "moving":
        rows[-1]["speed"] = "1.0"
    elif failure == "gap":
        rows[-1]["obstacle_distance"] = "1.0"
    with path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    (metric,) = analyze(tmp_path, make_plots=False)
    failures = {"collision", "lane"} if scenario == "curve_lane" else {"collision", "moving", "gap"}
    assert metric.success == (failure not in failures)
    report = (tmp_path / "summary.md").read_text()
    assert scenario in report
    assert "empty_lane" not in report and "obstacle_stop" not in report


@pytest.mark.parametrize("scenario", ["lead_vehicle", "lead_brake", "oncoming_overtake"])
@pytest.mark.parametrize("failure", [None, "collision", "gap", "moving", "early"])
def test_moving_success_rules(tmp_path, scenario, failure):
    from analysis.analyze_results import compute_metrics, load_rows

    rows = []
    for i, mode in enumerate(("BRAKE", "STOPPED", "AVOID", "PASSING", "RETURN", "CRUISE")):
        rows.append(dict(t=i, speed=0.0, lateral_offset=0.0, odometer=i,
                         obstacle_distance=8.0, collision=False, mode=mode,
                         gt_obstacle_distance=8.0, adjacent_lane_clear="1",
                         gt_adjacent_lane_clear="1", other_gap=[30, 20, 21, 30, 40, 50][i]))
    if failure == "collision":
        rows[-1]["collision"] = True
    if failure == "gap":
        rows[0]["obstacle_distance"] = 1.0
    if failure == "moving":
        rows[-1]["speed"] = 1.0
    if failure == "early":
        rows[2]["other_gap"] = 19.0
    path = tmp_path / f"{scenario}_run1.csv"
    with path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    metric = compute_metrics("Town01", scenario, 1, load_rows(path))
    failures = {"collision"}
    if scenario != "oncoming_overtake":
        failures.add("gap")
    if scenario == "lead_brake":
        failures.add("moving")
    if scenario == "oncoming_overtake":
        failures.add("early")
        assert metric.waited_for_oncoming == (failure != "early")
    assert metric.success == (failure not in failures)
    assert scenario in (analyze(tmp_path, make_plots=False)[0].scenario)


@pytest.mark.parametrize("gaps,expected", [([20, None], True), ([20, 19], False),
                                         ([20, 21], True), ([None, 21], False)])
def test_oncoming_wait_uses_first_avoid_only(gaps, expected):
    from analysis.analyze_results import waited_for_oncoming

    rows = [{"mode": "STOPPED", "other_gap": gaps[0]},
            {"mode": "AVOID", "other_gap": gaps[1]},
            {"mode": "AVOID", "other_gap": 30.0}]
    assert waited_for_oncoming(rows) == expected


def test_abort_counts_as_reaction_and_has_plot_color():
    from analysis.analyze_results import _reaction, MODE_COLORS

    rows = [dict(t=0.0, mode="CRUISE", obstacle_distance=25.0),
            dict(t=0.5, mode="ABORT", obstacle_distance=20.0)]
    assert _reaction(rows) == (20.0, 0.5)
    assert MODE_COLORS["ABORT"] == "#f6b26b"
