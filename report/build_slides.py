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
    s = new_slide(prs, "Architecture", "The architecture separates sensing, obstacle detection, decision making, and control, with ownership shown in the diagram. The obstacle interface supplies distance and adjacent-lane availability to the decision module. The diagram also shows the camera lane branch, which is evaluated separately in Run C. In the reported evaluations, lane steering and the LiDAR corridor use map information.")
    picture(s, "architecture_owners.png", 0.7, 1.35, 8.0, 5.45)
    box(s, "Perception → decision → control\n\nSmall module interfaces\n\nCamera steering: Run C\n\nFinal Run B: map steering", 9.0, 1.85, 3.6, 4.8, 20)
    s = new_slide(prs, "Obstacle detection", "A roof-mounted, 32-channel LiDAR supplies one sweep per tick. A range gate first filters the points, then a lane-shaped corridor from the map selects the relevant path. Road removal uses grade-limited height estimates in 2 m bins. A cluster must contain at least 3 points within 1 m to count as an obstacle.")
    box(s, "32-channel core / 64-channel moving · frame-synchronised", 0.8, 1.65, 11.8, 0.6, 24, True)
    stages = [("Range gate", "Filter the sweep"), ("Lane corridor", "Follow map geometry"), ("Road removal", "2 m bins\nGrade-limited"), ("Cluster check", "≥3 points\nWithin 1 m")]
    for i, (title, detail) in enumerate(stages):
        x = 0.75+i*3.15
        rule(s, x, 2.95, 2.45, ACCENT if i == 0 else INK)
        box(s, title, x, 3.25, 2.7, 0.7, 20, True)
        box(s, detail, x, 4.1, 2.7, 1.2)
        if i < 3:
            box(s, "→", x+2.7, 3.4, 0.4, 0.6, 20)
    s = new_slide(prs, "Decision logic", "The controller includes FOLLOW and ABORT alongside the modes, shown as the normal stop and avoidance branches. The stopping threshold combines reaction distance, braking distance, and a margin. Avoidance requires projected adjacent clearance and a 1 s debounce. Moving scenarios extend sensing to 180 m. An obstacle below 5 m triggers emergency braking in any mode, and braking preserves the manoeuvre state when needed.")
    box(s, "CRUISE", 0.8, 1.6, 2.2, 0.65, 24, True, ACCENT)
    box(s, "Blocked adjacent lane → BRAKE → STOPPED", 1.25, 2.5, 11.2, 0.6, 22)
    box(s, "Projected clearance + 1 s free → AVOID → PASSING → RETURN", 1.25, 3.35, 11.2, 0.65, 20)
    box(s, "Any mode · obstacle below 5 m → EMERGENCY_STOP", 1.25, 4.2, 11.2, 0.6, 20)
    rule(s, 0.8, 5.1, 11.7)
    box(s, "d_stop = v·t_react + v²/(2a) + margin", 0.8, 5.4, 11.8, 0.65, 26, True)
    box(s, "t_react = 1.0 s     a = 6 m/s²     margin = 4 m", 0.8, 6.2, 11.8, 0.5)


def evidence(prs: Presentation) -> None:
    s = new_slide(prs, "Test design", "Run A preserves 90 original ground-truth runs. Final Run B uses LiDAR obstacle and adjacent-lane detection with map steering. Core scenarios have 30 runs each, curve scenarios 20 each, lead scenarios 25 each and oncoming overtakes 20. Some town/scenario combinations are unavailable. Run C is a separate 18-run camera-steering evaluation; noise testing is a separate 15-run Town04 experiment.")
    box(s, "Run B: 8 scenarios across 6 towns · 200 runs", 0.8, 1.65, 11.7, 0.7, 28, True)
    box(s, "Core: empty lane / stop / avoid\nCurves: lane following / stopping (45–120°)\nMoving: lead vehicle / lead brake / oncoming overtake", 0.8, 2.65, 11.7, 2.0, 24)
    box(s, "Run A: 90 original ground-truth runs\nRun C: 18 camera + LiDAR runs\nNoise: 15 Town04 runs · LiDAR noise 0.05 m", 0.8, 5.0, 11.7, 1.45, 20)
    s = new_slide(prs, "Results", "Final Run B achieved 194/200 successes and zero collisions with map-waypoint steering. Four lead-brake runs did not fully stop within the time limit despite staying about 5–6 m behind. Two Town02_Opt oncoming runs failed. Run 2 has waited_for_oncoming=True; run 4 has False, contradicting a claim that both remained safe. Zero collisions does not establish safe lane use.")
    box(s, "200 runs · 194/200 successful · 0 collisions", 0.8, 1.45, 11.8, 0.7, 30, True, ACCENT)
    rows = [("Scenario", "Success", "Collisions"), ("empty_lane", "30/30", "0"), ("obstacle_stop", "30/30", "0"), ("obstacle_avoid", "30/30", "0"), ("curve_lane", "20/20", "0"), ("curve_stop", "20/20", "0"), ("lead_vehicle", "25/25", "0"), ("lead_brake", "21/25", "0"), ("oncoming_overtake", "18/20", "0")]
    for i, row in enumerate(rows):
        y = 2.35+i*0.40
        for x, width, value in zip([0.85, 6.6, 9.2], [5.5, 2.2, 2.4], row):
            box(s, value, x, y, width, 0.39, 17, i == 0)
    box(s, "Failures: 4 lead-brake timeouts + 2 oncoming; one oncoming safety flag is False", 0.8, 6.25, 11.8, 0.48, 17)
    s = new_slide(prs, "Moving traffic", "Gap tracking estimates obstacle speed. FOLLOW uses a desired gap of 6 m plus 1.8 s times ego speed and brakes below 3 s TTC. Possibly moving obstacles cannot be overtaken, including before motion confirmation. Adjacent clearance must remain free for 1 s; unsafe committed pull-outs ABORT back behind the obstacle when room allows. Low-speed pull-out control supports restarting after waiting. Moving scenarios use 64-channel 185 m LiDAR because an overtake needs approximately 150 m of visibly free lane.")
    box(s, "FOLLOW: gap = 6 m + 1.8 s × speed\nTTC <3 s → brake\nNever overtake a possibly moving lead\n\nOncoming: nearest − closing·9 s >10 m\n1 s free before pull-out · ABORT if blocked\n64 channels · 185 m LiDAR", 0.8, 1.6, 5.6, 4.65, 20)
    picture(s, "Town04_lead_vehicle_run1.png", 6.65, 1.7, 5.9, 4.6)
    box(s, "Run B · Town04 · FOLLOW", 6.65, 6.35, 5.9, 0.4, 17)
    s = new_slide(prs, "Camera lane detection", "The pipeline combines local-contrast white and yellow masks with curb edges, a wider ROI, boundary-pair checks and a lane-width prior. PD steering uses median filtering and a rate limit. Tuning used drives 0–1 of 1,674 labelled CARLA frames; held-out drive 2 has 558 frames. Detection is 91.2%, offset MAE 0.265 m rounded to 0.27 m, and steering-sign agreement 73.2%. MAE uses detected frames and sign metrics use absolute true offset above 0.2 m. Town01 and Town10HD remain weak. Run C achieved 17/18, with one collision after Town03_Opt empty_lane run 1 drifted off the road.")
    box(s, "White contrast + yellow + curb edges\nWider ROI → boundary-pair geometry\nLane-width prior → PD steering\nMedian filter + steering rate limit", 0.8, 1.65, 5.65, 2.9, 22)
    box(s, "Held-out test: drive 2\n91.2% detection\n0.27 m offset MAE\n73.2% steering-sign agreement", 7.0, 1.65, 5.5, 2.9, 23, True)
    box(s, "1,674 labelled frames · tune drives 0–1 / test drive 2\nClosed loop: 17/18 · 1 collision (Town03_Opt road departure)\nWeak offline towns: Town01 and Town10HD", 0.8, 5.05, 11.8, 1.45, 21)
    s = new_slide(prs, "Example runs", "All three plots are from final Run B. The curve-stop example shows braking on curved geometry. The lead-vehicle example shows FOLLOW. The oncoming example shows waiting before completing the pass and return. These runs use map steering, not camera steering.")
    for i, (name, title) in enumerate([("Town04_curve_stop_run1.png", "Town04 · curve stop"), ("Town04_lead_vehicle_run1.png", "Town04 · FOLLOW"), ("Town01_oncoming_overtake_run1.png", "Town01 · oncoming")]):
        x = 0.65+i*4.2
        box(s, title, x, 1.6, 4, 0.6, 20, True)
        picture(s, name, x, 2.35, 4, 4.05)


def closing(prs: Presentation) -> None:
    s = new_slide(prs, "What broke and how we fixed it", "The LiDAR corridor originally swept onto guardrails; it now follows map lane geometry. Stale sensor frames delayed braking under load; bounded current-frame reads fix this. Oncoming cars at 60–80 m were filtered as road after empty height bins; retaining the last height and extending LiDAR range supports early detection. Standard maps crashed the CARLA renderer, so Town02, Town03 and Town05 use layered _Opt variants in final Run B.")
    rows = [("LiDAR corridor hit guardrails", "Follow map lane geometry"), ("Stale sensor frames delayed braking", "Read the current simulation frame"), ("Oncoming cars lost at 60–80 m", "Keep height through empty bins; 185 m range"), ("CARLA renderer crashed on standard maps", "Use Town02/03/05_Opt variants")]
    for i, (problem, fix) in enumerate(rows):
        y = 1.75+i*1.2
        box(s, problem, 0.8, y, 5.6, 0.9, 21)
        box(s, "→", 6.45, y, 0.45, 0.6)
        box(s, fix, 7.0, y, 5.5, 0.9, 21)
        rule(s, 0.8, y+1.0, 11.7)
    s = new_slide(prs, "Limitations & next steps", "Run B includes selected moving traffic, but not pedestrians or dense traffic. Map steering and map corridor geometry remain dependencies. Camera testing is small and town-dependent. Noise testing is limited to one town and range-noise level; it achieved 15/15, with obstacle-distance MAE about 1.1 m. Junction navigation is not implemented and remains Khalid's E5 card. The final matrix uses _Opt maps for renderer compatibility.")
    box(s, "Current limits", 0.8, 1.6, 5.6, 0.6, 24, True)
    box(s, "Next steps", 7.0, 1.6, 5.5, 0.6, 24, True)
    pairs = [("Map steering in final Run B", "Improve camera robustness across towns"), ("Lead-brake / oncoming non-completions", "Improve stopping convergence and progress"), ("Noise: 15/15 in Town04 only", "Broader sensor / weather / traffic tests"), ("Junction navigation not implemented", "Khalid’s E5 card · junction routing")]
    for i, (limit, step) in enumerate(pairs):
        y = 2.5+i*1.0
        box(s, limit, 0.8, y, 5.6, 0.85, 20)
        box(s, step, 7.0, y, 5.5, 0.85, 20)
    s = new_slide(prs, "Demo / Q&A", "The recorded demo is in the GitHub release tagged v0.1-demo. It predates the final results. Building these documents does not start CARLA. Final results: Run B 194/200 with zero collisions; Run C 17/18 with one collision. Questions can address detection, control, test coverage and the remaining camera and navigation work.")
    box(s, "Final Run B: 194/200 · 0 collisions", 0.8, 1.8, 11.8, 0.8, 32, True, ACCENT)
    box(s, "Camera Run C: 17/18 · 1 collision", 0.8, 3.0, 11.8, 0.7, 28)
    box(s, "Recorded demo · GitHub release v0.1-demo\nEarlier demonstration; final evidence is Run B", 0.8, 4.4, 11.8, 1.2, 22)
    box(s, "Questions & discussion", 0.8, 6.0, 11.8, 0.6, 28, True)


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
    assert 12 <= len(prs.slides) <= 14
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

