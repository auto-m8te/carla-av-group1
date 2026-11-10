"""Findings & Results Analysis (Owner: Omar).

Reads results/<scenario>_run<N>.csv from scenarios/run_scenarios.py and produces:
    results/summary.md        - per-scenario table for the report
    results/per_run.csv       - one row of metrics per run
    results/plots/*.png       - speed / lateral offset / obstacle distance traces

Metrics (Trello card): success rate, lane-keeping error, reaction distance/time.

Usage:  python -m analysis.analyze_results --results results
"""

from __future__ import annotations

import argparse
import csv
import math
import logging
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import mean
from typing import Optional

LANE_DEPARTURE_M = 1.0   # |offset| above this while cruising = left the lane
MIN_STOP_GAP_M = 2.0
RETURNED_TOL_M = 0.5
log = logging.getLogger(__name__)
SCENARIO_ORDER = ("empty_lane", "obstacle_stop", "obstacle_avoid", "curve_lane", "curve_stop",
                  "lead_vehicle", "lead_brake", "oncoming_overtake")
FILE_RE = re.compile(r"^(" + "|".join(SCENARIO_ORDER) + r")_run(\d+)\.csv$")


@dataclass(frozen=True)
class RunMetrics:
    town: str
    scenario: str
    run: int
    success: bool
    collision: bool
    duration_s: float
    lane_rms_m: float
    lane_max_m: float
    reaction_distance_m: Optional[float]
    reaction_time_s: Optional[float]
    min_gap_m: Optional[float]
    final_offset_m: float
    perception_mae_m: Optional[float] = None   # |detected - ground truth| gap, ticks where both see it
    perception_missed: int = 0                 # ticks where ground truth sees an obstacle, detector not
    perception_false: int = 0                  # ticks where detector sees one, ground truth not
    adjacent_disagree: int = 0                 # ticks where adjacent-lane-free differs from truth
    waited_for_oncoming: bool = False


def load_rows(path: Path) -> list[dict]:
    with path.open(newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise ValueError(f"{path} is empty")
    for r in rows:
        for k in ("t", "speed", "lateral_offset", "odometer"):
            r[k] = float(r[k])
        r["obstacle_distance"] = float(r["obstacle_distance"]) if r["obstacle_distance"] else None
        r["collision"] = r["collision"] in ("1", "True")
        gt = r.get("gt_obstacle_distance")
        r["gt_obstacle_distance"] = float(gt) if gt else None
        gt_adj = r.get("gt_adjacent_lane_clear")
        r["gt_adjacent_lane_clear"] = r.get("adjacent_lane_clear") if not gt_adj else gt_adj
        for key in ("obstacle_speed", "other_speed", "other_gap"):
            value = r.get(key)
            r[key] = float(value) if value not in (None, "") else None
    return rows


def _reaction(rows: list[dict]) -> tuple[Optional[float], Optional[float]]:
    """Distance to obstacle and time since first detection when we first left CRUISE."""
    first_seen = next((r for r in rows if r["obstacle_distance"] is not None), None)
    first_act = next((r for r in rows if r["mode"] != "CRUISE"), None)
    if first_seen is None or first_act is None:
        return None, None
    return first_act["obstacle_distance"], first_act["t"] - first_seen["t"]


def perception_error(rows: list[dict]) -> tuple[Optional[float], int, int]:
    """Compare logged obstacle_distance with ground truth (meaningful for runs that
    use a real perception module; ground-truth runs give 0 error)."""
    both = [abs(r["obstacle_distance"] - r["gt_obstacle_distance"]) for r in rows
            if r["obstacle_distance"] is not None and r["gt_obstacle_distance"] is not None]
    missed = sum(1 for r in rows if r["gt_obstacle_distance"] is not None and r["obstacle_distance"] is None)
    false = sum(1 for r in rows if r["obstacle_distance"] is not None and r["gt_obstacle_distance"] is None)
    return (mean(both) if both else None), missed, false


def waited_for_oncoming(rows: list[dict]) -> bool:
    """At first pull-out, the scripted car must be absent or moving away."""
    for index, row in enumerate(rows):
        if row["mode"] != "AVOID":
            continue
        gap = row.get("other_gap")
        if gap in (None, ""):
            return True
        previous = rows[index - 1].get("other_gap") if index else None
        return previous not in (None, "") and float(gap) > float(previous)
    return False


def _successful(scenario: str, rows: list[dict], min_gap: Optional[float],
                lane_max: float, collision: bool) -> bool:
    if collision:
        return False
    if scenario in ("empty_lane", "curve_lane"):
        return lane_max < LANE_DEPARTURE_M
    if scenario == "lead_vehicle":
        return min_gap is not None and min_gap >= MIN_STOP_GAP_M
    if scenario in ("obstacle_stop", "curve_stop", "lead_brake"):
        return rows[-1]["speed"] < 0.2 and (min_gap or 0) >= MIN_STOP_GAP_M
    modes = {r["mode"] for r in rows}
    completed = ({"AVOID", "PASSING", "RETURN"} <= modes
                 and abs(rows[-1]["lateral_offset"]) < RETURNED_TOL_M)
    return completed and (scenario != "oncoming_overtake" or waited_for_oncoming(rows))


def compute_metrics(town: str, scenario: str, run: int, rows: list[dict]) -> RunMetrics:
    cruise = [abs(r["lateral_offset"]) for r in rows if r["mode"] == "CRUISE"] or [0.0]
    gaps = [r["obstacle_distance"] for r in rows if r["obstacle_distance"] is not None]
    collision = any(r["collision"] for r in rows)
    last = rows[-1]
    reaction_d, reaction_t = _reaction(rows)
    min_gap = min(gaps) if gaps else None
    lane_max = max(cruise)
    mae, missed, false = perception_error(rows)
    adj_disagree = sum(1 for r in rows if r.get("adjacent_lane_clear") != r["gt_adjacent_lane_clear"])

    success = _successful(scenario, rows, min_gap, lane_max, collision)

    return RunMetrics(
        town=town,
        scenario=scenario,
        run=run,
        success=success,
        collision=collision,
        duration_s=last["t"],
        lane_rms_m=math.sqrt(mean(x * x for x in cruise)),
        lane_max_m=lane_max,
        reaction_distance_m=reaction_d,
        reaction_time_s=reaction_t,
        min_gap_m=min_gap,
        final_offset_m=last["lateral_offset"],
        perception_mae_m=mae,
        perception_missed=missed,
        perception_false=false,
        adjacent_disagree=adj_disagree,
        waited_for_oncoming=scenario == "oncoming_overtake" and waited_for_oncoming(rows),
    )


def _fmt(values: list[Optional[float]]) -> str:
    vals = [v for v in values if v is not None]
    if not vals:
        return "-"
    if len(vals) == 1:
        return f"{vals[0]:.2f}"
    return f"{mean(vals):.2f} ({min(vals):.2f}-{max(vals):.2f})"


def summary_table(metrics: list[RunMetrics]) -> str:
    lines = [
        "| Scenario | Runs | Success rate | Collisions | Lane RMS err (m) | Lane max err (m) "
        "| Reaction dist (m) | Reaction time (s) | Min gap (m) |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for sc in SCENARIO_ORDER:
        ms = [m for m in metrics if m.scenario == sc]
        if not ms:
            continue
        ok = sum(m.success for m in ms)
        lines.append(
            f"| {sc} | {len(ms)} | {ok}/{len(ms)} ({100 * ok / len(ms):.0f}%) "
            f"| {sum(m.collision for m in ms)} | {_fmt([m.lane_rms_m for m in ms])} "
            f"| {_fmt([m.lane_max_m for m in ms])} | {_fmt([m.reaction_distance_m for m in ms])} "
            f"| {_fmt([m.reaction_time_s for m in ms])} | {_fmt([m.min_gap_m for m in ms])} |"
        )
    return "\n".join(lines) + "\n"


def town_table(metrics: list[RunMetrics]) -> str:
    """Success count per town x scenario."""
    scenarios = [sc for sc in SCENARIO_ORDER if any(m.scenario == sc for m in metrics)]
    lines = [
        "| Town | " + " | ".join(scenarios) + " | Collisions |",
        "|---|" + "---|" * (len(scenarios) + 1),
    ]
    for town in sorted({m.town for m in metrics}):
        cells = []
        for sc in scenarios:
            ms = [m for m in metrics if m.town == town and m.scenario == sc]
            cells.append(f"{sum(m.success for m in ms)}/{len(ms)}" if ms else "-")
        collisions = sum(m.collision for m in metrics if m.town == town)
        lines.append(f"| {town} | " + " | ".join(cells) + f" | {collisions} |")
    return "\n".join(lines) + "\n"


def perception_table(metrics: list[RunMetrics]) -> str:
    """Obstacle-detector accuracy vs ground truth, per scenario."""
    lines = ["| Scenario | Distance MAE (m) | Missed ticks | False-alarm ticks | Adjacent-lane disagree ticks |",
             "|---|---|---|---|---|"]
    for sc in SCENARIO_ORDER:
        ms = [m for m in metrics if m.scenario == sc]
        if ms:
            lines.append(f"| {sc} | {_fmt([m.perception_mae_m for m in ms])} "
                         f"| {sum(m.perception_missed for m in ms)} | {sum(m.perception_false for m in ms)} "
                         f"| {sum(m.adjacent_disagree for m in ms)} |")
    return "\n".join(lines) + "\n"


def full_report(metrics: list[RunMetrics]) -> str:
    report = "### Overall\n\n" + summary_table(metrics) + "\n### Per town\n\n" + town_table(metrics)
    if any(m.perception_mae_m or m.perception_missed or m.perception_false or m.adjacent_disagree
           for m in metrics):
        report += "\n### Obstacle detection vs ground truth\n\n" + perception_table(metrics)
    return report


def plot_runs(runs: dict[str, list[dict]], out_dir: Path) -> list[Path]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)
    made = []
    panels = (("speed", "Speed (m/s)"), ("lateral_offset", "Lateral offset (m)"),
              ("obstacle_distance", "Obstacle distance (m)"))
    for name, rows in sorted(runs.items()):
        fig, axes = plt.subplots(3, 1, figsize=(7, 6.5), sharex=True)
        t = [r["t"] for r in rows]
        for ax, (key, label) in zip(axes, panels):
            ax.plot(t, [r[key] if r[key] is not None else float("nan") for r in rows], lw=1.6)
            ax.set_ylabel(label)
            ax.grid(alpha=0.3)
        _shade_modes(axes, rows)
        axes[-1].set_xlabel("Time (s)")
        fig.suptitle(name.replace("_", " "))
        fig.tight_layout()
        path = out_dir / f"{name}.png"
        fig.savefig(path, dpi=150)
        plt.close(fig)
        made.append(path)
    return made


MODE_COLORS = {"FOLLOW": "#ffe599", "BRAKE": "#f4b183", "STOPPED": "#e06666", "EMERGENCY_STOP": "#990000",
               "ABORT": "#f6b26b", "AVOID": "#9fc5e8", "PASSING": "#6fa8dc", "RETURN": "#b6d7a8"}


def _shade_modes(axes, rows: list[dict]) -> None:
    """Shade background by controller mode so the decision is visible on the plot."""
    start = 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or rows[i]["mode"] != rows[start]["mode"]:
            color = MODE_COLORS.get(rows[start]["mode"])
            if color:
                for ax in axes:
                    ax.axvspan(rows[start]["t"], rows[i - 1]["t"], color=color, alpha=0.35, lw=0)
            start = i


def analyze(results_dir: Path, make_plots: bool = True) -> list[RunMetrics]:
    runs, metrics = {}, []
    for path in sorted(results_dir.rglob("*.csv")):
        m = FILE_RE.match(path.name)
        if not m:
            continue
        town = path.parent.name if path.parent != results_dir else "-"
        rows = load_rows(path)
        runs[f"{town}_{path.stem}"] = rows
        metrics.append(compute_metrics(town, m.group(1), int(m.group(2)), rows))
    if not metrics:
        raise FileNotFoundError(f"No scenario CSVs in {results_dir}")

    with (results_dir / "per_run.csv").open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(asdict(metrics[0])))
        writer.writeheader()
        writer.writerows(asdict(m) for m in metrics)
    (results_dir / "summary.md").write_text(full_report(metrics), encoding="utf-8")
    if make_plots:
        plot_runs(runs, results_dir / "plots")
    return metrics


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results", default="results")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()
    metrics = analyze(Path(args.results), make_plots=not args.no_plots)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    log.info("%s", full_report(metrics))


if __name__ == "__main__":
    main()
