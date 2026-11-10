"""Draw the system architecture figure (report Fig. 1, E6 card).

Usage: python -m analysis.architecture_diagram --out ../report/figures/architecture.png [--owners]
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch  # noqa: E402

LAYER_COLORS = {
    "sim": "#e8eef7", "sensor": "#eef3e2", "perception": "#fff2d6",
    "decision": "#fde2dd", "control": "#e6e0f3", "tools": "#eeeeee",
}
EDGE = "#333333"


@dataclass(frozen=True)
class Box:
    key: str
    x: float
    y: float
    w: float
    h: float
    title: str
    body: str
    layer: str
    owner: str = ""


BOXES = (
    Box("carla", 0.5, 8.6, 11.0, 1.1, "CARLA 0.9.16 (synchronous, 20 Hz)",
        "6 towns · road network from OpenDRIVE map · parked-car scenarios", "sim", "Prithiv"),
    Box("cam", 0.5, 6.7, 5.2, 1.2, "RGB camera", "front-facing image each tick", "sensor", "Adil"),
    Box("lidar", 6.3, 6.7, 5.2, 1.2, "Roof LiDAR (32 ch)",
        "+2° .. −24°, 70 m, one 360° sweep per tick", "sensor", "Abir"),
    Box("lane", 0.5, 4.5, 5.2, 1.6, "Lane module (OpenCV)",
        "HSV mask → Canny → ROI → Hough\n→ lateral + heading error", "perception", "Adil"),
    Box("obst", 6.3, 4.5, 5.2, 1.6, "Obstacle module",
        "range gate → lane corridor (map)\n→ road removal → cluster check", "perception", "Abir"),
    Box("dec", 2.0, 2.3, 8.0, 1.6, "Decision: stop-or-avoid state machine",
        "CRUISE · BRAKE · STOPPED · EMERGENCY_STOP · AVOID · PASSING · RETURN\n"
        "d_stop = v·t_react + v²/2a + margin", "decision", "Omar"),
    Box("ctl", 2.0, 0.3, 8.0, 1.4, "Control",
        "PD lateral steering · feed-forward + P speed → throttle / steer / brake",
        "control", "Omar"),
    Box("log", 12.3, 4.5, 3.2, 5.2, "Test harness",
        "3 scenarios × 6 towns\n× 5 seeded runs\n\nper-tick CSV log\n+ ground truth\n\n"
        "metrics, plots,\ndetector error", "tools", "Omar"),
)

# Single-column (IEEE) version: no harness box, shorter text, bigger fonts.
COMPACT_BODY = {
    "carla": "synchronous 20 Hz · 6 towns",
    "cam": "front image", "lidar": "+2°..−24°, 70 m",
    "lane": "HSV → Canny → ROI → Hough", "obst": "lane corridor → road\nremoval → clusters",
    "dec": "CRUISE · BRAKE · STOPPED · E-STOP\nAVOID · PASSING · RETURN",
    "ctl": "PD steering · FF + P speed",
}
COMPACT_TITLE = {"carla": "CARLA 0.9.16", "lidar": "Roof LiDAR", "lane": "Lane module",
                 "dec": "Stop-or-avoid state machine"}

ARROWS = (
    ("carla", "cam", ""), ("carla", "lidar", ""),
    ("cam", "lane", ""), ("lidar", "obst", ""),
    ("lane", "dec", "steering_angle"), ("obst", "dec", "obstacle_distance\nadjacent_lane_clear"),
    ("dec", "ctl", "mode + target"),
)


def _centre_bottom(b: Box) -> tuple[float, float]:
    return b.x + b.w / 2, b.y


def _centre_top(b: Box) -> tuple[float, float]:
    return b.x + b.w / 2, b.y + b.h


def _draw_box(ax, b: Box, owners: bool, scale: float = 1.0) -> None:
    ax.add_patch(FancyBboxPatch((b.x, b.y), b.w, b.h, boxstyle="round,pad=0.02,rounding_size=0.15",
                                fc=LAYER_COLORS[b.layer], ec=EDGE, lw=1.2))
    top = b.y + b.h - 0.32
    ax.text(b.x + b.w / 2, top, b.title, ha="center", va="center", fontsize=10.5 * scale, weight="bold")
    ax.text(b.x + b.w / 2, top - 0.38 * scale, b.body, ha="center", va="top", fontsize=8.6 * scale,
            linespacing=1.3)
    if owners and b.owner:
        ax.text(b.x + b.w - 0.12, b.y + 0.12, b.owner, ha="right", va="bottom", fontsize=8,
                style="italic", color="#555555")


def _draw_arrow(ax, a: Box, b: Box, label: str, scale: float = 1.0) -> None:
    start, end = _centre_bottom(a), _centre_top(b)
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=14, lw=1.3, color=EDGE))
    if label:
        mx, my = (start[0] + end[0]) / 2, (start[1] + end[1]) / 2
        ax.text(mx + 0.12, my, label, fontsize=8.2 * scale, family="monospace", va="center",
                bbox=dict(fc="white", ec="none", pad=0.6))


def _compact(b: Box) -> Box:
    return replace(b, body=COMPACT_BODY.get(b.key, b.body), title=COMPACT_TITLE.get(b.key, b.title))


def draw(out: Path, owners: bool, compact: bool = False) -> Path:
    shown = tuple(_compact(b) for b in BOXES if b.key != "log") if compact else BOXES
    boxes = {b.key: b for b in shown}
    scale = 1.45 if compact else 1.0
    fig, ax = plt.subplots(figsize=(6.4, 6.6) if compact else (10, 6.6))
    ax.set_xlim(0, 12 if compact else 16)
    ax.set_ylim(0, 10)
    ax.axis("off")
    for b in shown:
        _draw_box(ax, b, owners, scale)
    for a, b, label in ARROWS:
        _draw_arrow(ax, boxes[a], boxes[b], label, scale)
    # control acts on the simulated vehicle; harness reads every tick
    ax.annotate("", xy=(0.5, 9.0), xytext=(2.0, 1.0), arrowprops=dict(
        arrowstyle="-|>", lw=1.3, color=EDGE, connectionstyle="angle,angleA=180,angleB=-90,rad=0"))
    ax.text(0.15, 5.0, "VehicleControl", rotation=90, fontsize=8.2 * scale, family="monospace",
            va="center")
    if not compact:
        ax.annotate("", xy=(12.3, 5.3), xytext=(11.5, 5.3),
                    arrowprops=dict(arrowstyle="<|-|>", lw=1.0, color="#777777", ls="--"))
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=220)
    plt.close(fig)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default="../report/figures/architecture.png")
    ap.add_argument("--owners", action="store_true", help="label each block with its owner (Trello/slides)")
    ap.add_argument("--compact", action="store_true", help="single-column version for the IEEE report")
    args = ap.parse_args()
    print(draw(Path(args.out), args.owners, args.compact))


if __name__ == "__main__":
    main()
