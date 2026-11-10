"""Build the Group 1 presentation without starting CARLA."""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from PIL import Image
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.util import Inches, Pt

ROOT = Path(__file__).resolve().parent
INK = "1d2733"
ACCENT = "d9480f"
FONT = "Segoe UI"
OUTLINE: list[tuple[str, str]] = []


def box(slide: Any, text: str, x: float, y: float, w: float, h: float,
        size: int = 20, bold: bool = False, color: str = INK) -> Any:
    shape = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    frame = shape.text_frame
    frame.word_wrap = True
    frame.margin_left = frame.margin_right = Inches(0.03)
    frame.margin_top = frame.margin_bottom = Inches(0.03)
    for i, line in enumerate(text.split("\n")):
        p = frame.paragraphs[0] if i == 0 else frame.add_paragraph()
        p.text = line
        p.font.name, p.font.size = FONT, Pt(size)
        p.font.bold, p.font.color.rgb = bold, RGBColor.from_string(color)
        p.space_after = Pt(4)
        p.line_spacing = 1.12
    return shape


def rule(slide: Any, x: float, y: float, w: float, color: str = INK) -> None:
    shape = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(x), Inches(y),
                                   Inches(w), Inches(0.018))
    shape.fill.solid()
    shape.fill.fore_color.rgb = RGBColor.from_string(color)
    shape.line.fill.background()


def new_slide(prs: Presentation, title: str, notes: str) -> Any:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    slide.background.fill.solid()
    slide.background.fill.fore_color.rgb = RGBColor(255, 255, 255)
    box(slide, title, 0.6, 0.32, 12.1, 0.65, 32, True)
    rule(slide, 0.63, 1.13, 12.05, ACCENT)
    box(slide, "GROUP 1  /  SRH BERLIN  /  WS2026/27", 0.63, 6.98, 10, 0.4, 14)
    box(slide, str(len(prs.slides)), 12.15, 6.98, 0.5, 0.4, 14)
    slide.notes_slide.notes_text_frame.text = notes
    OUTLINE.append((title, notes))
    return slide


def picture(slide: Any, filename: str, x: float, y: float, w: float, h: float) -> None:
    path = ROOT / "figures" / filename
    assert path.is_file(), f"Missing image: {path}"
    with Image.open(path) as img:
        ratio = img.width / img.height
    width, height = min(w, h * ratio), min(h, w / ratio)
    slide.shapes.add_picture(str(path), Inches(x + (w-width)/2),
                             Inches(y + (h-height)/2), Inches(width), Inches(height))


def intro(prs: Presentation) -> None:
    s = new_slide(prs, "Autonomous driving in CARLA", "We are Group 1 at SRH Berlin, presenting our WS2026/27 project for Prof. A. Laboudi. Our vehicle follows a lane, detects obstacles, and chooses whether to stop or steer around them. The implementation runs in CARLA 0.9.16. We will explain the modules, the evaluation, and the limitations of the current system.")
    box(s, "Follow the lane.\nDetect the obstacle.\nChoose a safe response.", 0.7, 1.85, 11.6, 2.6, 32, True)
    box(s, "CARLA 0.9.16  ·  Group 1", 0.7, 4.85, 11, 0.5, 20)
    box(s, "Khalid · Abir · Adil · Prithiv · Omar\nSRH Berlin · WS2026/27 · Prof. A. Laboudi", 0.7, 5.65, 11.5, 0.9, 20)
    s = new_slide(prs, "Problem & goals", "The task combines perception, decision making, and vehicle control in a simulated city. The first requirement is to follow a driving lane. The second is to detect obstacles, and the third is to avoid collisions by stopping or steering around them. The fourth is safe navigation, with junction navigation still part of the future work.")
    for y, heading, detail in [(1.65, "Follow a driving lane", "Keep the vehicle aligned with the road"), (2.85, "Detect obstacles", "Identify objects along the driving path"), (4.05, "Avoid collisions", "Stop or steer around when the adjacent lane is free"), (5.25, "Navigate safely", "Coordinate perception, decisions and control")]:
        box(s, heading, 0.8, y, 11.8, 0.45, 24, True)
        box(s, detail, 0.8, y+0.5, 11.8, 0.45)
    s = new_slide(prs, "Team & roles", "Prithiv owns simulation and integration, while Adil owns lane detection and steering. Abir develops the LiDAR obstacle detector. Omar owns collision avoidance, testing, and documentation. Khalid joined later and contributes city navigation and innovation.")
    roles = [("Prithiv", "Simulation & integration"), ("Adil", "Lane detection & steering"), ("Abir", "Obstacle detection · LiDAR"), ("Omar", "Collision avoidance, testing & documentation"), ("Khalid", "City navigation & innovation · joined later")]
    for i, (name, role) in enumerate(roles):
        y = 1.65 + i * 0.95
        box(s, name, 0.8, y, 2, 0.5, 24, True)
        box(s, role, 3.0, y+0.04, 9.5, 0.6)
        rule(s, 0.8, y+0.72, 11.6)


def methods(prs: Presentation) -> None:
    s = new_slide(prs, "Architecture", "The architecture separates sensing, obstacle detection, decision making, and control, with ownership shown in the diagram. The obstacle interface supplies distance and adjacent-lane availability to the decision module. The diagram also shows the intended camera lane branch, which is still pending integration. In the reported evaluations, lane steering and the LiDAR corridor use map information.")
    picture(s, "architecture_owners.png", 0.7, 1.35, 8.0, 5.45)
    box(s, "Perception → decision → control\n\nSmall module interfaces\n\nCamera lane integration pending\n\nEvaluated steering: map ground truth", 9.0, 1.85, 3.6, 4.8, 20)
    s = new_slide(prs, "Obstacle detection", "A roof-mounted, 32-channel LiDAR supplies one sweep per tick. A range gate first filters the points, then a lane-shaped corridor from the map selects the relevant path. Road removal uses grade-limited height estimates in 2 m bins. A cluster must contain at least 3 points within 1 m to count as an obstacle.")
    box(s, "32-channel roof LiDAR · one sweep per tick", 0.8, 1.65, 11.8, 0.6, 24, True)
    stages = [("Range gate", "Filter the sweep"), ("Lane corridor", "Follow map geometry"), ("Road removal", "2 m bins\nGrade-limited"), ("Cluster check", "≥3 points\nWithin 1 m")]
    for i, (title, detail) in enumerate(stages):
        x = 0.75+i*3.15
        rule(s, x, 2.95, 2.45, ACCENT if i == 0 else INK)
        box(s, title, x, 3.25, 2.7, 0.7, 20, True)
        box(s, detail, x, 4.1, 2.7, 1.2)
        if i < 3:
            box(s, "→", x+2.7, 3.4, 0.4, 0.6, 20)
    s = new_slide(prs, "Decision logic", "The controller uses seven modes, shown as the normal stop and avoidance branches. The stopping threshold combines reaction distance, braking distance, and a margin. Avoidance is allowed only if the adjacent lane is free over 60 m. An obstacle below 5 m triggers emergency braking in any mode, and braking preserves the manoeuvre state when needed.")
    box(s, "CRUISE", 0.8, 1.6, 2.2, 0.65, 24, True, ACCENT)
    box(s, "Blocked adjacent lane → BRAKE → STOPPED", 1.25, 2.5, 11.2, 0.6, 22)
    box(s, "Free over 60 m → AVOID → PASSING → RETURN → CRUISE", 1.25, 3.35, 11.2, 0.65, 20)
    box(s, "Any mode · obstacle below 5 m → EMERGENCY_STOP", 1.25, 4.2, 11.2, 0.6, 20)
    rule(s, 0.8, 5.1, 11.7)
    box(s, "d_stop = v·t_react + v²/(2a) + margin", 0.8, 5.4, 11.8, 0.65, 26, True)
    box(s, "t_react = 1.0 s     a = 6 m/s²     margin = 4 m", 0.8, 6.2, 11.8, 0.5)


def evidence(prs: Presentation) -> None:
    s = new_slide(prs, "Test design", "We test across six towns with three scenarios and five seeded runs for each combination. The scenarios are an empty lane, an obstacle with a blocked adjacent lane, and an obstacle with a free adjacent lane. The same seed set is evaluated once with ground-truth obstacle perception and once with LiDAR. Lane steering remains based on map ground truth in both evaluations.")
    box(s, "6 towns × 3 scenarios × 5 seeded runs", 0.8, 1.7, 11.7, 0.8, 30, True)
    box(s, "90 runs per perception source", 0.8, 2.65, 11.7, 0.65, 28, True, ACCENT)
    for i, text in enumerate(["Empty lane → follow the lane", "Obstacle stop → adjacent lane blocked", "Obstacle avoid → adjacent lane free"]):
        box(s, text, 0.85, 3.65+i*0.7, 11.7, 0.6)
    box(s, "Repeated with ground truth and LiDAR · same seeds", 0.8, 6.15, 11.7, 0.6, 20, True)
    s = new_slide(prs, "Results", "Both perception configurations completed all 90 runs successfully with no collisions. LiDAR distance MAE was 0.48 m for stopping and 0.44 m for avoidance. False alarms occurred on 136 of 23,715 ticks, or 0.6 percent, and none changed vehicle behaviour. These outcomes apply to the tested static-obstacle scenarios with ground-truth lane steering.")
    for x, name in [(0.85, "Ground truth"), (7.0, "LiDAR")]:
        box(s, name, x, 1.55, 5.4, 0.5, 24, True)
        box(s, "90/90", x, 2.2, 5.4, 0.85, 42, True, ACCENT)
        box(s, "successful runs", x, 3.13, 5.4, 0.5)
        box(s, "0 collisions", x, 3.8, 5.4, 0.6, 30, True, ACCENT)
        rule(s, x, 4.65, 5.35)
    box(s, "Obstacle source: actor geometry\nDistance MAE: reference\nFalse alarms: reference", 0.85, 4.95, 5.4, 1.7)
    box(s, "Distance MAE: 0.48 m stop / 0.44 m avoid\nFalse alarms: 136 / 23,715 ticks (0.6%)\nBehaviour changes from false alarms: none", 7.0, 4.95, 5.4, 1.7, 18)
    s = new_slide(prs, "Example runs", "These plots illustrate the ground-truth evaluation runs rather than the LiDAR distance-error measurements. Town01 demonstrates an avoidance manoeuvre through the empty oncoming lane. Town03 demonstrates braking and holding stopped behind an obstacle. Town04 demonstrates avoidance on a multi-lane highway, with the manoeuvre returning to the original lane.")
    plots = [("Town01_obstacle_avoid_run1.png", "Town01 · avoid"), ("Town03_obstacle_stop_run1.png", "Town03 · stop"), ("Town04_obstacle_avoid_run2.png", "Town04 · avoid")]
    for i, (filename, caption) in enumerate(plots):
        x = 0.65+i*4.2
        box(s, caption, x, 1.6, 4.0, 0.55, 20, True)
        picture(s, filename, x, 2.3, 4.0, 4.05)


def closing(prs: Presentation) -> None:
    s = new_slide(prs, "What broke and how we fixed it", "CARLA testing revealed twelve problems that were fixed during development; this slide selects four. Road and lane identifiers changed between segments, so adjacency checks moved to geometry. LiDAR guardrail detections were reduced by following the lane shape, and elevated-road errors were addressed by estimating road height near the path only. Limiting the corridor to the end of the path reduced false-alarm ticks from 1,564 to 136.")
    rows = [("Road/lane IDs changed between segments", "Geometric lateral-offset check"), ("LiDAR corridor swept onto a guardrail", "Lane-shaped corridor from the map"), ("Elevated highway looked like an obstacle", "Road height from points near the path"), ("Corridor reached trees beyond road turns", "Stop at path end · false alarms 1,564 → 136")]
    box(s, "Observed problem", 0.8, 1.55, 5.6, 0.5, 20, True)
    box(s, "Fix", 7.0, 1.55, 5.5, 0.5, 20, True)
    for i, (problem, fix) in enumerate(rows):
        y = 2.35+i*1.05
        box(s, problem, 0.8, y, 5.7, 0.75)
        box(s, "→", 6.45, y, 0.45, 0.6)
        box(s, fix, 7.0, y, 5.5, 0.8)
        rule(s, 0.8, y+0.89, 11.7)
    s = new_slide(prs, "Limitations & next steps", "Lane steering still uses ground truth, and Adil's camera lane module is the next integration step. The current experiments use static obstacles, so moving traffic requires time-to-collision reasoning. The LiDAR is noise-free, and steep crests in Town05 can still produce false alarms. Further work includes sensor noise and city navigation through junctions, with Khalid contributing to navigation.")
    box(s, "Current limits", 0.8, 1.6, 5.6, 0.6, 24, True)
    box(s, "Next steps", 7.0, 1.6, 5.5, 0.6, 24, True)
    pairs = [("Ground-truth lane steering", "Integrate Adil’s camera lane module"), ("Static obstacles only", "Moving obstacles + time-to-collision"), ("Noise-free LiDAR", "Introduce sensor noise"), ("Town05: steep-crest false alarms", "City junction navigation · Khalid")]
    for i, (limit, step) in enumerate(pairs):
        y = 2.6+i*0.9
        box(s, limit, 0.8, y, 5.6, 0.7)
        box(s, step, 7.0, y, 5.5, 0.7)
    s = new_slide(prs, "Demo / Q&A", "The recorded demonstration is available in the GitHub release tagged v0.1-demo. The command shown runs a Town04 demonstration with LiDAR obstacle and adjacent-lane detection, a HUD, and real-time pacing. It is provided for use with a running CARLA environment; building this presentation does not start CARLA. We welcome questions about detection, decision logic, testing, and remaining integration work.")
    box(s, "Recorded demo", 0.8, 1.65, 11.8, 0.55, 24, True)
    box(s, "GitHub release · v0.1-demo", 0.8, 2.3, 11.8, 0.6, 24)
    box(s, "Live demo command", 0.8, 3.4, 11.8, 0.55, 24, True)
    box(s, "python -m scenarios.run_scenarios --towns Town04 --runs 1\n--realtime --hud --obstacle lidar --adjacent lidar --out results_demo", 0.8, 4.15, 11.8, 1.0, 18)
    box(s, "Questions & discussion", 0.8, 5.8, 11.8, 0.65, 30, True)


def estimated_height(shape: Any, size: int) -> float:
    frame = shape.text_frame
    width = (shape.width-frame.margin_left-frame.margin_right)/12700
    total = 0.0
    for p in frame.paragraphs:
        # Conservative mean glyph width; include explicit breaks and wrapping.
        for line in p.text.split("\n"):
            lines = max(1, math.ceil(len(line)*size*0.56/max(width, 1)))
            total += lines*size*1.18
        total += 4
    return total


def validate(prs: Presentation) -> None:
    assert 10 <= len(prs.slides) <= 12
    for slide in prs.slides:
        assert slide.notes_slide.notes_text_frame.text.strip()
        for shape in slide.shapes:
            assert shape.left >= 0 and shape.top >= 0
            assert shape.left+shape.width <= prs.slide_width+10
            assert shape.top+shape.height <= prs.slide_height+10
            if not shape.has_text_frame or not shape.text.strip():
                continue
            frame = shape.text_frame
            available = (shape.height-frame.margin_top-frame.margin_bottom)/12700
            size = int(frame.paragraphs[0].font.size.pt)
            while estimated_height(shape, size) >= available and size > 14:
                size -= 1
            for p in frame.paragraphs:
                p.font.size = Pt(size)
            assert estimated_height(shape, size) < available, shape.text


def main() -> None:
    OUTLINE.clear()
    prs = Presentation()
    prs.slide_width, prs.slide_height = Inches(13.333333), Inches(7.5)
    for build in (intro, methods, evidence, closing):
        build(prs)
    validate(prs)
    prs.save(ROOT / "Group1_Presentation.pptx")
    content = "# Group 1 presentation — speaker notes\n\n"
    content += "\n\n".join(f"## {i}. {title}\n\n{notes}" for i, (title, notes) in enumerate(OUTLINE, 1))
    content += "\n\nAll requested facts placed in slides or speaker notes.\n"
    (ROOT / "slides_outline.md").write_text(content, encoding="utf-8")
    print(f"Built and validated {len(prs.slides)} slides: {ROOT / 'Group1_Presentation.pptx'}")


if __name__ == "__main__":
    main()

