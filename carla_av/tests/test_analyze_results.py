"""Feeds toy-sim traces (same CSV format as CARLA runs) through the analysis."""

import csv

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
