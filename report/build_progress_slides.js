// Group 1 progress deck — explains what the team built so far.
const path = require("path");
const pptxgen = require("pptxgenjs");
const { applyTheme } = require("C:/Users/omara/.claude/skills/synced/304d8bde-e861-4ec4-aa12-7bf38db266ae_34808be3-08fc-4cb2-bd13-69ab0c1e528c/pptx/scripts/apply_theme.js");

const ROOT = "C:/Users/omara/Desktop/Uni/Engineering teamwork 3";
const FIG = (f) => path.join(ROOT, "report/figures", f);
const OUT = path.join(ROOT, "report/Group1_Progress_Presentation.pptx");

const HEX = {
  asphalt: "1F2329", white: "FFFFFF", ink: "2E3440", fog: "F3F4F6",
  yellow: "F2B705", green: "2E8B57", red: "C0392B", blue: "3B6EA8", gray: "7A8291", orange: "E67E22",
  line: "D9DCE1",
};
const THEME = {
  name: "Group1 Asphalt",
  headFontFace: "Arial",
  bodyFontFace: "Calibri",
  colors: {
    dk1: HEX.asphalt, lt1: HEX.white, dk2: HEX.ink, lt2: HEX.fog,
    accent1: HEX.yellow, accent2: HEX.green, accent3: HEX.red, accent4: HEX.blue,
    accent5: HEX.gray, accent6: HEX.orange, hlink: HEX.blue, folHlink: HEX.gray,
  },
};

const fs = require("fs");
const RESULTS_JSON = path.join(ROOT, "report/figures/prithiv_eval.json");
const NEW_RESULTS = fs.existsSync(RESULTS_JSON) ? JSON.parse(fs.readFileSync(RESULTS_JSON, "utf8")) : null;

const pres = new pptxgen();
pres.layout = "LAYOUT_16x9"; // 10 x 5.625
pres.title = "Group 1 — Autonomous Driving in CARLA: Progress";
pres.author = "Group 1";
pres.theme = { headFontFace: THEME.headFontFace, bodyFontFace: THEME.bodyFontFace };
const C = pres.SchemeColor;

// ---------- layouts ----------
pres.defineSlideMaster({
  title: "DARK_TITLE",
  background: { color: C.text1 },
  objects: [
    { placeholder: { options: { name: "title", type: "title", x: 0.6, y: 1.55, w: 8.8, h: 1.5, fontSize: 38, bold: true, color: C.background1, valign: "bottom", align: "left", margin: 0 }, text: "" } },
    { placeholder: { options: { name: "body", type: "body", x: 0.6, y: 3.2, w: 8.8, h: 1.2, fontSize: 16, color: C.background2, valign: "top", margin: 0 }, text: "" } },
  ],
});
pres.defineSlideMaster({
  title: "CONTENT",
  background: { color: C.background1 },
  objects: [
    { placeholder: { options: { name: "title", type: "title", x: 0.5, y: 0.28, w: 9.0, h: 0.7, fontSize: 28, bold: true, color: C.text1, valign: "middle", align: "left", margin: 0 }, text: "" } },
    { text: { text: "Group 1 · Autonomous driving in CARLA · Slides made with Claude Code", options: { x: 0.5, y: 5.22, w: 6, h: 0.25, fontSize: 9, color: C.accent5, margin: 0 } } },
  ],
  slideNumber: { x: 9.0, y: 5.22, w: 0.5, h: 0.25, fontSize: 9, color: C.accent5, align: "right" },
});

let sec = "";
function section(title) { sec = title; pres.addSection({ title }); }
function content(title) {
  const s = pres.addSlide({ masterName: "CONTENT", sectionTitle: sec });
  s.addText(title, { placeholder: "title" });
  return s;
}
function tag(s, text, x, y, fill = C.accent1, color = C.text1, w) {
  s.addText(text, {
    shape: pres.shapes.ROUNDED_RECTANGLE, rectRadius: 0.12, x, y, w: w || 0.12 + text.length * 0.085, h: 0.28,
    fill: { color: fill }, color, fontSize: 10, bold: true, align: "center", valign: "middle", margin: 0, isTextBox: true,
    objectName: "tag " + text,
  });
}
function card(s, x, y, w, h, fill = C.background2) {
  s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, rectRadius: 0.08, fill: { color: fill }, line: { color: fill }, objectName: "card" });
}
function txt(s, text, o) { s.addText(text, Object.assign({ isTextBox: true, margin: 0, valign: "top", color: C.text2, fontSize: 14 }, o)); }
function bullets(s, items, o) {
  const runs = items.map((t, i) => ({ text: t, options: { bullet: { indent: 14 }, breakLine: i < items.length - 1, paraSpaceAfter: 6 } }));
  s.addText(runs, Object.assign({ isTextBox: true, margin: 0, valign: "top", color: C.text2, fontSize: 14 }, o));
}

// ---------- 1. Title ----------
section("Intro");
{
  const s = pres.addSlide({ masterName: "DARK_TITLE", sectionTitle: sec });
  tag(s, "SPRINT 1 · PROGRESS REPORT", 0.6, 1.05, C.accent1, C.text1, 2.7);
  s.addText("Autonomous driving in CARLA: what we built, why and how", { placeholder: "title" });
  s.addText("Group 1 · Khalid · Abir · Adil · Prithiv · Omar\nEngineering Teamwork WS2026/27 · SRH Berlin · Prof. A. Laboudi · 17 Nov 2026\nSlides made with Claude Code", { placeholder: "body" });
  // lane-marking motif
  for (let i = 0; i < 6; i++) s.addShape(pres.shapes.RECTANGLE, { x: 0.6 + i * 1.55, y: 4.95, w: 0.9, h: 0.08, fill: { color: C.accent1 }, line: { color: C.accent1 }, objectName: "lane dash" });
  s.addNotes("We are Group 1. This deck explains what we have built so far in Sprint 1: the goal, how we organised the work, each module of the car, how we tested it, the results, what went wrong along the way, and what is left.");
}

// ---------- 2. Goal ----------
{
  const s = content("The task: a self-driving car in a simulated city");
  txt(s, "Build, as a team, a vehicle in the CARLA 0.9.16 simulator that can:", { x: 0.5, y: 1.1, w: 9, h: 0.35 });
  const goals = [
    ["1", "Follow its lane", "Keep to the centre of the driving lane, on straight roads and curves"],
    ["2", "Detect obstacles", "See parked cars, slow or braking cars and oncoming traffic with LiDAR"],
    ["3", "Stop or steer around", "Choose: brake to a safe stop, or overtake when the next lane is free"],
    ["4", "Navigate the city", "Plan a route on the road map and drive it through junctions and turns"],
  ];
  goals.forEach((g, i) => {
    const x = 0.5 + i * 2.33;
    card(s, x, 1.6, 2.0, 2.75);
    s.addText(g[0], { shape: pres.shapes.OVAL, x: x + 0.2, y: 1.8, w: 0.55, h: 0.55, fill: { color: C.text1 }, color: C.accent1, fontSize: 20, bold: true, align: "center", valign: "middle", margin: 0, isTextBox: true, objectName: "goal num" });
    txt(s, g[1], { x: x + 0.2, y: 2.5, w: 1.65, h: 0.6, fontSize: 16, bold: true, color: C.text1 });
    txt(s, g[2], { x: x + 0.2, y: 3.1, w: 1.65, h: 1.15, fontSize: 12 });
  });
  tag(s, "STATUS: ALL FOUR WORKING IN SIMULATION", 0.5, 4.6, C.accent2, C.background1, 3.6);
  s.addNotes("The brief has four parts. Lane following, obstacle detection, collision avoidance by stopping or steering around, and safe navigation. All four now run in CARLA; the rest of the deck shows how and how well.");
}

// ---------- 3. Progress at a glance ----------
{
  const s = content("Progress at a glance");
  const stats = [
    ["6 / 6", "epics done (E1–E6)", C.text1],
    ["413", "simulated test runs in CARLA", C.text1],
    ["194/200", "final LiDAR evaluation passed, 0 collisions", C.accent2],
    ["25", "bugs found in CARLA and fixed", C.text1],
  ];
  stats.forEach((st, i) => {
    const x = 0.5 + i * 2.33;
    card(s, x, 1.15, 2.0, 1.85);
    txt(s, st[0], { x: x + 0.15, y: 1.35, w: 1.75, h: 0.75, fontSize: 32, bold: true, color: st[2], fontFace: "Arial" });
    txt(s, st[1], { x: x + 0.15, y: 2.15, w: 1.75, h: 0.75, fontSize: 12 });
  });
  txt(s, "Sprint 1 goal (10–24 Nov)", { x: 0.5, y: 3.3, w: 4.3, h: 0.3, fontSize: 14, bold: true, color: C.text1 });
  txt(s, "Car drives in CARLA, keeps its lane on the city map, and raises an \"obstacle ahead\" signal.", { x: 0.5, y: 3.65, w: 4.3, h: 1.2, fontSize: 13 });
  txt(s, "Where we are now", { x: 5.2, y: 3.3, w: 4.3, h: 0.3, fontSize: 14, bold: true, color: C.text1 });
  bullets(s, [
    "Goal reached and exceeded",
    "Stop / avoid, moving traffic, camera lanes and junction navigation also done",
    "Prithiv's ROS 2 work reviewed; 14 ideas merged",
  ], { x: 5.2, y: 3.65, w: 4.3, h: 1.4, fontSize: 13 });
  s.addNotes("Headline numbers. All six epics are done. We ran 413 test drives in CARLA across all evaluations; only one of them ended in a collision, in the small camera-steering test. The final LiDAR evaluation passed 194 of 200 runs with zero collisions. Along the way we found and fixed 25 problems that only showed up in the simulator. Sprint 1 asked only for driving, lane keeping and an obstacle signal; we went beyond that.");
}

// ---------- 4. Team ----------
section("Team & process");
{
  const s = content("Team, roles and how we work");
  const rows = [
    ["Prithiv", "Simulation, Trello owner; ROS 2 Lanebot", "E1, E5"],
    ["Adil", "Lane detection & steering; literature review", "E2"],
    ["Abir", "Obstacle detection (sensors); methodology", "E3"],
    ["Omar", "Collision avoidance, testing, docs, repo", "E4, E6"],
    ["Khalid", "City navigation & innovation (joined later)", "E5"],
  ];
  const hdr = (t) => ({ text: t, options: { bold: true, color: C.background1, fill: { color: C.text1 } } });
  s.addTable([[hdr("Member"), hdr("Role"), hdr("Epics")], ...rows.map((r) => r.map((c) => ({ text: c })))], {
    x: 0.5, y: 1.15, w: 5.3, colW: [1.0, 3.5, 0.8], fontSize: 12, color: C.text2, border: { type: "solid", pt: 0.75, color: HEX.line }, rowH: 0.42, margin: 0.06, objectName: "roles table",
  });
  const ways = [
    ["Trello", "One card per task; a card is Done only with evidence (CSV, plot, video)"],
    ["GitHub", "Branch + PR per card; tests must pass"],
    ["WhatsApp", "Blockers posted the same day"],
    ["Stand-ups", "Done · next · blocked; review on 24 Nov"],
  ];
  ways.forEach((w, i) => {
    const y = 1.15 + i * 0.95;
    card(s, 6.1, y, 3.4, 0.8);
    txt(s, w[0], { x: 6.25, y: y + 0.08, w: 3.1, h: 0.27, fontSize: 13, bold: true, color: C.text1 });
    txt(s, w[1], { x: 6.25, y: y + 0.36, w: 3.1, h: 0.42, fontSize: 11 });
  });
  s.addNotes("Five members, each owning one or two epics on the Trello board. Roles were a starting point: when a card was blocked, someone else picked it up so the whole system could be integrated and tested. Definition of Done: runs on the team CARLA machine, code in the repo with passing tests, shown working in CARLA with evidence attached, and documented in the README.");
}

// ---------- 5. Timeline ----------
{
  const s = content("How the project grew, step by step");
  const steps = [
    ["Base car", "Lane following + LiDAR obstacle stop / avoid"],
    ["Safer LiDAR", "Adjacent-lane check, false-alarm fixes, first 90-run test"],
    ["Harder traffic", "Sharp curves, moving lead car, oncoming cars"],
    ["Camera lanes", "OpenCV lane detector tuned on 1,674 frames"],
    ["Final evaluation", "200 LiDAR runs, camera + noise tests"],
    ["City navigation", "Route planner through junctions, 60 runs"],
    ["Prithiv's ideas", "Traffic lights, ambulances, pedestrians, dashboard"],
  ];
  const w = 1.18, gap = 0.12;
  steps.forEach((st, i) => {
    const x = 0.5 + i * (w + gap);
    s.addText(String(i + 1), { shape: pres.shapes.OVAL, x: x + w / 2 - 0.25, y: 1.3, w: 0.5, h: 0.5, fill: { color: C.accent1 }, color: C.text1, fontSize: 16, bold: true, align: "center", valign: "middle", margin: 0, isTextBox: true, objectName: "step" });
    if (i < steps.length - 1) s.addShape(pres.shapes.LINE, { x: x + w / 2 + 0.3, y: 1.55, w: w + gap - 0.6, h: 0, line: { color: C.accent5, width: 1.5, endArrowType: "triangle" }, objectName: "arrow" });
    card(s, x, 2.0, w, 2.0);
    txt(s, st[0], { x: x + 0.1, y: 2.12, w: w - 0.2, h: 0.55, fontSize: 13, bold: true, color: C.text1 });
    txt(s, st[1], { x: x + 0.1, y: 2.7, w: w - 0.2, h: 1.25, fontSize: 11 });
  });
  txt(s, "Each step was merged only after it ran in CARLA and its tests passed. 277 automated tests (run without CARLA).", { x: 0.5, y: 4.3, w: 9, h: 0.6, fontSize: 13 });
  s.addNotes("This is the order in which the system grew, following the git history. We started with a basic lane-following car that stops or avoids a parked obstacle, then made LiDAR detection reliable, added harder scenarios with moving traffic, added the camera lane detector, ran the final evaluations, and finally added navigation through junctions.");
}

// ---------- 6. Architecture ----------
section("What we built");
{
  const s = content("System architecture");
  s.addImage({ path: FIG("architecture_owners.png"), x: 0.5, y: 1.05, w: 5.9, h: 5.9 * 1452 / 2200, altText: "Architecture: CARLA feeds camera and LiDAR; lane and obstacle modules feed the stop-or-avoid decision, which drives control", objectName: "architecture" });
  txt(s, "Three layers", { x: 6.7, y: 1.1, w: 2.8, h: 0.3, fontSize: 15, bold: true, color: C.text1 });
  bullets(s, [
    "Perception: camera finds the lane, LiDAR finds obstacles",
    "Decision: a state machine picks cruise, follow, stop or avoid",
    "Control: steering, throttle and brake every 0.05 s",
  ], { x: 6.7, y: 1.5, w: 2.8, h: 2.1, fontSize: 12 });
  txt(s, "Small agreed interfaces let us build modules in parallel:", { x: 6.7, y: 3.55, w: 2.8, h: 0.5, fontSize: 12 });
  txt(s, "steering_angle\nobstacle_distance\nadjacent_lane_clear", { x: 6.7, y: 4.05, w: 2.8, h: 0.8, fontSize: 12, fontFace: "Courier New", color: C.text1 });
  s.addNotes("CARLA runs synchronously at 20 Hz. The camera feeds the lane module, the LiDAR feeds the obstacle module. Both hand small values to the decision module: a steering angle, the distance to the nearest obstacle in our path, and whether the neighbouring lane is free. The decision module chooses a mode and the controller turns it into throttle, steer and brake. The test harness logs everything with ground truth.");
}

// ---------- 7. LiDAR ----------
{
  const s = content("E3 · Seeing obstacles with LiDAR");
  const steps = ["Range gate", "Lane-shaped corridor from the map", "Remove the road surface", "Cluster check: ≥3 points, ≥0.25 m tall", "obstacle_distance"];
  steps.forEach((st, i) => {
    const y = 1.15 + i * 0.72;
    const last = i === steps.length - 1;
    s.addText(st, { shape: pres.shapes.ROUNDED_RECTANGLE, rectRadius: 0.08, x: 0.5, y, w: 4.0, h: 0.52, fill: { color: last ? C.text1 : C.background2 }, color: last ? C.accent1 : C.text1, fontSize: 13, bold: true, align: "left", valign: "middle", margin: 8, isTextBox: true, fontFace: last ? "Courier New" : undefined, objectName: "lidar step" });
    if (!last) s.addShape(pres.shapes.LINE, { x: 2.5, y: y + 0.52, w: 0, h: 0.2, line: { color: C.accent5, width: 1.5, endArrowType: "triangle" }, objectName: "arrow" });
  });
  txt(s, "Why it works", { x: 5.0, y: 1.15, w: 4.5, h: 0.3, fontSize: 15, bold: true, color: C.text1 });
  bullets(s, [
    "Only looks where the car will actually drive, so guardrails, trees and buildings beside the road are ignored",
    "Also checks the neighbouring lane, including how fast an oncoming car is closing",
    "Every choice came from a false alarm we saw in CARLA",
  ], { x: 5.0, y: 1.55, w: 4.5, h: 2.0, fontSize: 13 });
  card(s, 5.0, 3.75, 2.1, 1.2); card(s, 7.4, 3.75, 2.1, 1.2);
  txt(s, "0.3–0.5 m", { x: 5.15, y: 3.85, w: 1.9, h: 0.5, fontSize: 24, bold: true, color: C.text1, fontFace: "Arial" });
  txt(s, "distance error vs ground truth", { x: 5.15, y: 4.4, w: 1.9, h: 0.5, fontSize: 11 });
  txt(s, "1,564 → 136", { x: 7.55, y: 3.85, w: 1.9, h: 0.5, fontSize: 20, bold: true, color: C.accent2, fontFace: "Arial" });
  txt(s, "false-alarm ticks after the corridor fix", { x: 7.55, y: 4.4, w: 1.9, h: 0.5, fontSize: 11 });
  s.addNotes("A roof LiDAR gives one 360-degree sweep per tick. We keep points in range, keep only those inside a corridor shaped like our lane from the map, remove the road surface, and accept an obstacle only as a cluster of at least three points that stands at least 25 cm tall. The result is a distance to the nearest obstacle. Distance error against ground truth is about 0.3 to 0.5 m. Fixing the corridor cut false alarms in the first 90-run test from 1,564 ticks to 136.");
}

// ---------- 8. Decision ----------
{
  const s = content("E4 · Deciding: stop, follow or steer around");
  const modes = [
    ["CRUISE", C.accent4, "drive the lane at target speed"],
    ["FOLLOW", C.accent4, "keep a safe gap behind a moving car"],
    ["BRAKE → STOPPED", C.accent3, "next lane blocked: stop before the obstacle"],
    ["AVOID → PASSING → RETURN", C.accent2, "next lane free: overtake and come back"],
    ["ABORT", C.accent6, "next lane closes mid-manoeuvre: go back behind"],
    ["EMERGENCY_STOP", C.accent3, "anything under 5 m: full brake, in any mode"],
  ];
  modes.forEach((m, i) => {
    const y = 1.15 + i * 0.6;
    s.addText(m[0], { shape: pres.shapes.ROUNDED_RECTANGLE, rectRadius: 0.08, x: 0.5, y, w: 2.7, h: 0.46, fill: { color: m[1] }, color: C.background1, fontSize: 11, bold: true, align: "center", valign: "middle", margin: 0, isTextBox: true, objectName: "mode" });
    txt(s, m[2], { x: 3.35, y, w: 2.65, h: 0.46, fontSize: 12, valign: "middle" });
  });
  card(s, 6.2, 1.15, 3.3, 3.75, C.text1);
  txt(s, "When to react", { x: 6.4, y: 1.3, w: 2.9, h: 0.3, fontSize: 14, bold: true, color: C.accent1 });
  txt(s, "d = v·t + v²/2a + margin", { x: 6.4, y: 1.7, w: 2.9, h: 0.5, fontSize: 13, color: C.background1, fontFace: "Courier New" });
  bullets(s, [
    "Faster car → reacts earlier",
    "Overtake starts early enough to finish the lane change",
    "Never overtakes something that may be moving",
    "Next lane must stay free for 1 s before pulling out",
  ], { x: 6.4, y: 2.35, w: 2.9, h: 2.5, fontSize: 12, color: C.background1 });
  s.addNotes("The decision module is a state machine. Normally it cruises. If an obstacle appears it checks the neighbouring lane: if it is blocked, it brakes to a stop; if it is free, it overtakes and returns. Behind a moving car it switches to FOLLOW, like adaptive cruise control. If the target lane closes during a pull-out it aborts back behind the obstacle. Under 5 m there is always an emergency stop. The braking point grows with speed: reaction distance plus braking distance plus a margin.");
}

// ---------- 9. Moving traffic ----------
{
  const s = content("Moving traffic: follow, wait, then overtake");
  s.addImage({ path: FIG("Town01_oncoming_overtake_run1.png"), x: 0.5, y: 1.05, w: 4.1, h: 4.1 * 975 / 1050, altText: "Town01 oncoming overtake run: speed, lateral offset and obstacle distance over time", objectName: "oncoming plot" });
  txt(s, "What the plot shows (Town01)", { x: 4.95, y: 1.1, w: 4.55, h: 0.3, fontSize: 15, bold: true, color: C.text1 });
  bullets(s, [
    "Car stops ~16 m behind a parked car (red band)",
    "Waits ~11 s while oncoming traffic passes",
    "Pulls into the other lane (offset to −3.7 m), passes, returns to the centre (green band)",
  ], { x: 4.95, y: 1.5, w: 4.55, h: 1.7, fontSize: 13 });
  txt(s, "Rules for moving cars", { x: 4.95, y: 3.25, w: 4.55, h: 0.3, fontSize: 15, bold: true, color: C.text1 });
  bullets(s, [
    "FOLLOW gap: 6 m + 1.8 s × speed",
    "Lane free only if: distance − closing speed × 9 s > 10 m",
    "Needs ~150 m visible free lane → 64-channel, 185 m LiDAR",
  ], { x: 4.95, y: 3.65, w: 4.55, h: 1.4, fontSize: 13 });
  s.addNotes("This is a real run from the final evaluation. The car approaches a parked car, stops, and waits while oncoming traffic passes. When the lane has been free long enough, it pulls out, passes and returns to the lane centre. For moving traffic we track how fast the gap changes. The oncoming check asks whether the lane stays free for the whole manoeuvre, which told us we need about 150 m of visible lane, so moving scenarios use a longer-range LiDAR.");
}

// ---------- 10. Camera ----------
{
  const s = content("E2 · Finding the lane with a camera");
  bullets(s, [
    "OpenCV: white, yellow and curb masks → edges → lane-boundary pair",
    "Checks: pair geometry and expected lane width",
    "PD steering with median filter and rate limit",
    "Tuned on 1,674 auto-labelled CARLA frames; tested on a held-out drive (558 frames)",
  ], { x: 0.5, y: 1.15, w: 4.2, h: 2.4, fontSize: 13 });
  const stats = [["91.2%", "lanes detected"], ["0.27 m", "offset error"], ["17/18", "closed-loop runs"]];
  stats.forEach((st, i) => {
    const x = 0.5 + i * 1.45;
    card(s, x, 3.65, 1.3, 1.25);
    txt(s, st[0], { x: x + 0.1, y: 3.75, w: 1.15, h: 0.5, fontSize: 20, bold: true, color: C.text1, fontFace: "Arial" });
    txt(s, st[1], { x: x + 0.1, y: 4.3, w: 1.15, h: 0.5, fontSize: 11 });
  });
  s.addChart(pres.charts.BAR, [{ name: "Detection", labels: ["Town01", "Town02", "Town03", "Town04", "Town05", "Town10HD"], values: [92.5, 88.2, 95.7, 90.3, 90.3, 90.3] }], {
    x: 5.0, y: 1.1, w: 4.5, h: 2.3, barDir: "col", chartColors: [HEX.blue],
    showTitle: true, title: "Held-out detection rate by town (%)", titleFontSize: 12, titleColor: HEX.ink, titleFontFace: "+mn-lt",
    showValue: true, dataLabelPosition: "outEnd", dataLabelFontSize: 10, dataLabelColor: HEX.ink, dataLabelFontFace: "+mn-lt", dataLabelFormatCode: "0.0",
    valAxisHidden: true, valAxisMinVal: 0, valAxisMaxVal: 110, valGridLine: { style: "none" }, catGridLine: { style: "none" },
    catAxisLabelColor: HEX.gray, catAxisLabelFontSize: 10, catAxisLabelFontFace: "+mn-lt", showLegend: false, objectName: "camera chart",
  });
  card(s, 5.0, 3.65, 4.5, 1.25);
  txt(s, "Still weak", { x: 5.15, y: 3.75, w: 4.2, h: 0.3, fontSize: 13, bold: true, color: C.accent3 });
  txt(s, "Town01 and Town10HD (offset and steering direction). One closed-loop run in Town03 drifted off the road.", { x: 5.15, y: 4.08, w: 4.2, h: 0.8, fontSize: 12 });
  s.addNotes("The camera lane detector finds the two lane boundaries with colour and curb masks, checks they form a plausible pair with the expected lane width, and turns the offset into steering with a PD controller. We tuned it on 1,674 automatically labelled frames and tested on a held-out drive: 91 percent detection and 0.27 m average offset error. Driving with camera steering plus LiDAR passed 17 of 18 runs. Town01 and Town10HD remain weak, and one Town03 run drifted off the road.");
}

// ---------- 11. Navigation ----------
{
  const s = content("E5 · Navigating the city through junctions");
  s.addImage({ path: path.join(ROOT, "carla_av/results_nav2/plots/Town01_route_obstacle_run1.png"), x: 0.5, y: 1.05, w: 4.1, h: 4.1 * 975 / 1050, altText: "Town01 route with a parked obstacle: car drives the route, then stops 10 m before the obstacle", objectName: "route plot" });
  bullets(s, [
    "Route planner: shortest path on the map's road graph",
    "Path follower: Stanley steering (from Prithiv)",
    "Slows to 5 m/s at junctions, 4 m/s in sharp turns",
    "Same LiDAR safety layer follows the route; no overtaking near junctions",
  ], { x: 4.95, y: 1.1, w: 4.55, h: 2.2, fontSize: 13 });
  const stats = [["30/30", "stopped for an obstacle on the route"], ["29/30", "routes reached the goal"], ["0", "collisions in 60 runs"]];
  stats.forEach((st, i) => {
    const x = 4.95 + i * 1.55;
    card(s, x, 3.55, 1.4, 1.35);
    txt(s, st[0], { x: x + 0.1, y: 3.65, w: 1.2, h: 0.5, fontSize: 20, bold: true, color: i === 1 ? C.text1 : C.accent2, fontFace: "Arial" });
    txt(s, st[1], { x: x + 0.1, y: 4.18, w: 1.2, h: 0.7, fontSize: 11 });
  });
  s.addNotes("The newest part. A planner finds the shortest route on the map's road graph, and a path follower drives it (now Stanley steering, from Prithiv), slowing down at junctions and sharp turns. The obstacle corridor follows the route, so the same detection and stop logic protects the car along the way. In 60 runs over six towns the car stopped for every obstacle, reached the goal in 29 of 30 routes, and never collided. Under a strict 1.5 m offset rule only 18 of 30 routes pass, because the car still cuts some corners at junction exits.");
}

// ---------- Why we chose what ----------
section("Why we chose what");
function choiceSlide(title, rows, notes) {
  const s = content(title);
  const labels = ["WE CHOSE", "INSTEAD OF", "BECAUSE"];
  const colX = [0.7, 3.35, 5.75], colW = [2.45, 2.2, 3.6];
  rows.forEach((r, i) => {
    const y = 1.1 + i * 1.32;
    card(s, 0.5, y, 9.0, 1.18);
    r.forEach((t, j) => {
      txt(s, labels[j], { x: colX[j], y: y + 0.1, w: colW[j], h: 0.22, fontSize: 9, bold: true, color: j === 0 ? C.accent2 : C.accent5, charSpacing: 1 });
      txt(s, t, { x: colX[j], y: y + 0.34, w: colW[j], h: 0.8, fontSize: j === 0 ? 13 : 12, bold: j === 0, color: j === 0 ? C.text1 : C.text2 });
    });
  });
  s.addNotes(notes);
}
choiceSlide("Perception: why LiDAR, map and camera", [
  ["CARLA Python API, synchronous 20 Hz", "ROS 2 bridge, wall-clock timing", "Every run is seeded and repeatable, runs on our Windows machine, and 277 tests run without a simulator"],
  ["LiDAR for obstacles", "Camera-only detection", "Measures distance directly (0.3–0.5 m error), independent of light and colour"],
  ["Map-shaped search corridor", "Fixed box in front of the car", "A box sweeps onto guardrails and buildings in curves; false alarms fell from 1,564 to 136 ticks"],
], "Our perception choices. We drive CARLA directly from Python in synchronous mode, so every run is repeatable with a seed and we can test the logic without a simulator. Obstacles come from LiDAR because it measures distance directly. And we only search inside a corridor shaped like the lane from the map, because a simple box in front of the car caused hundreds of false alarms on curves.");
choiceSlide("Decision and control: why rules and Stanley", [
  ["Rule-based state machine", "Learned (end-to-end) policy", "Every decision traces to one rule; no training data needed; each rule unit-tested"],
  ["Speed-based stopping distance", "Fixed trigger distance", "A fixed trigger was too short at speed (Town01 bug); v·t + v²/2a + margin scales with speed"],
  ["Stanley steering on routes (from Prithiv)", "Pure pursuit", "Pure pursuit aims at a point ahead and cuts corners at junction exits; Stanley tracks the path at the front axle"],
], "Decision and control. A rule-based state machine, not a learned policy, so that every decision can be explained and tested, and we need no training data. The braking point grows with speed, because a fixed trigger distance failed at higher speeds. And for routes we switched from pure pursuit to Stanley steering, an idea from Prithiv's controller, because pure pursuit cuts corners at junction exits.");

// ---------- Prithiv's Lanebot ----------
section("Prithiv's Lanebot");
{
  const s = content("Prithiv's Lanebot: a rule-following driver");
  const nodes = [
    ["CARLA + ROS bridge", 0.5, 1.2, C.text1, C.background1],
    ["lane_node\nwhere is our lane?", 0.5, 2.25, C.background2, C.text1],
    ["obstacle_node\nLiDAR: own + left lane", 0.5, 3.15, C.background2, C.text1],
    ["signals_node\nlights, limits, sirens", 0.5, 4.05, C.background2, C.text1],
    ["control_node\nStanley + StVO rules", 3.4, 3.15, C.accent1, C.text1],
  ];
  nodes.forEach((n) => s.addText(n[0], { shape: pres.shapes.ROUNDED_RECTANGLE, rectRadius: 0.08, x: n[1], y: n[2], w: 2.5, h: 0.72, fill: { color: n[3] }, color: n[4], fontSize: 11, bold: true, align: "center", valign: "middle", margin: 4, isTextBox: true, objectName: "node" }));
  // Node -> control_node arrows. Shape heights must be positive: upward arrows use flipV.
  const arrow = (o) => s.addShape(pres.shapes.LINE, Object.assign({ line: { color: C.accent5, width: 1.25, endArrowType: "triangle" }, objectName: "arrow" }, o));
  arrow({ x: 3.0, y: 2.61, w: 0.4, h: 0.69 });
  arrow({ x: 3.0, y: 3.51, w: 0.4, h: 0.01 });
  arrow({ x: 3.0, y: 3.72, w: 0.4, h: 0.69, flipV: true });
  s.addShape(pres.shapes.LINE, { x: 1.75, y: 1.92, w: 0, h: 0.33, line: { color: C.accent5, width: 1.25, endArrowType: "triangle" }, objectName: "arrow" });
  txt(s, "ROS 2 Jazzy · Ubuntu · Town04 motorway", { x: 3.4, y: 1.25, w: 2.6, h: 0.6, fontSize: 12, italic: true, color: C.accent5 });
  txt(s, "German traffic rules built in", { x: 6.4, y: 1.15, w: 3.1, h: 0.3, fontSize: 14, bold: true, color: C.text1 });
  bullets(s, [
    "Keep right; overtake only on the left, across a broken line",
    "Indicator on 3 s before any lane change",
    "2-second gap; speed limits and signs",
    "Stop at red and amber lights",
    "Make way for ambulances and police",
    "Dashboard window + 10-min rule checker",
  ], { x: 6.4, y: 1.55, w: 3.1, h: 3.4, fontSize: 12 });
  s.addNotes("Prithiv built a separate version of the car on ROS 2 under Ubuntu. Each job is its own node: one finds the lane from the map, one checks the LiDAR for cars in our lane and the left lane, one reads traffic lights, speed signs and emergency vehicles, and the control node drives with a Stanley steering controller and German traffic rules. It also has a dashboard window and a rule-checking test. It drives the Town04 motorway.");
}
{
  const s = content("What we took from Prithiv, and how");
  const hdr = (t) => ({ text: t, options: { bold: true, color: C.background1, fill: { color: C.text1 } } });
  const rows = [
    ["Stanley steering + curve feed-forward", "Route follower (now default)", "Stops corner cutting at junction exits"],
    ["Stop at red / amber lights (StVO §37)", "Red light = parked obstacle at stop line", "Routes used to run red lights"],
    ["Left lane must continue (left_room)", "Check target lane along the manoeuvre", "Never pull into a lane that ends"],
    ["Broken-line rule (StVO §5)", "Optional flag --obey-markings", "Legal overtakes where needed"],
    ["Indicators before lane changes", "Blinkers in AVOID / RETURN / YIELD", "Shows intent in CARLA and videos"],
    ["Make way for ambulances (StVO §38)", "New YIELD mode: right edge, half speed", "Emergency vehicles get through"],
    ["Pedestrians crossing", "LiDAR stops the car; no overtaking near them", "Protect vulnerable road users"],
    ["Instrument cluster", "Dashboard window fed every tick (--cluster)", "Shows what the car sees and decides"],
  ];
  s.addTable([[hdr("Prithiv's idea"), hdr("How it is in our car"), hdr("Why")], ...rows.map((r) => r.map((c, j) => ({ text: c, options: { bold: j === 0 } })))], {
    x: 0.5, y: 1.05, w: 9.0, colW: [3.1, 3.2, 2.7], fontSize: 11, color: C.text2, border: { type: "solid", pt: 0.75, color: HEX.line }, rowH: 0.4, margin: 0.06, objectName: "adopt table",
  });
  txt(s, "Continued on the next slide. Ported into our pipeline (not as ROS nodes); 277 unit tests.", { x: 0.5, y: 4.82, w: 9, h: 0.3, fontSize: 11, italic: true, color: C.accent5 });
  s.addNotes("These ideas came from Prithiv's work. Each one fixes a gap we had: corner cutting on routes, no traffic-light handling, the risk of pulling into a lane that ends, no emergency vehicles or pedestrians, and no live view of what the car decides. We ported the ideas into our own pipeline rather than running his ROS nodes, and wrote unit tests for each.");
}
{
  const s = content("What we took from Prithiv (2): rules and realism");
  const hdr = (t) => ({ text: t, options: { bold: true, color: C.background1, fill: { color: C.text1 } } });
  const rows = [
    ["Speed limits at 90 % (StVO §3)", "Map limit caps our cruise speed", "9 of 200 old runs went over 33 km/h; towns are 30 zones"],
    ["Slow down before lower-limit signs", "Look 100 m ahead for speed signs", "Arrive at the new limit, not after it"],
    ["\"Fahren auf Sicht\"", "Never faster than we can stop within sensor range", "Safe if the LiDAR sees less"],
    ["Mixed traffic (traffic_node)", "--traffic N: lorry, police, cars via Traffic Manager", "Tests the car among other road users"],
    ["Night + headlight dipping", "--night: high beam, dipped for cars within 200 m", "Realistic night driving"],
    ["Rule checker (monitor.py)", "Logs speeding, off-road, near misses every tick", "Measures legal driving, not just crashes"],
  ];
  s.addTable([[hdr("Prithiv's idea"), hdr("How it is in our car"), hdr("Why")], ...rows.map((r) => r.map((c, j) => ({ text: c, options: { bold: j === 0 } })))], {
    x: 0.5, y: 1.05, w: 9.0, colW: [2.9, 3.4, 2.7], fontSize: 11, color: C.text2, border: { type: "solid", pt: 0.75, color: HEX.line }, rowH: 0.48, margin: 0.06, objectName: "adopt table 2",
  });
  card(s, 0.5, 4.5, 9.0, 0.55);
  txt(s, "Check run, Town03 at night with 12 traffic cars: 5/5 scenarios, 0 collisions, 0 s speeding, 0 s off-road, 0 near misses.", { x: 0.7, y: 4.6, w: 8.6, h: 0.35, fontSize: 12, bold: true, color: C.accent2, valign: "middle" });
  s.addNotes("The second half of what we took. Speed rules from his control node: 90 percent of the limit, slowing before lower-limit signs, and never faster than we can stop within what the sensor sees. Before this, 9 of our 200 final runs went above 33 km/h in what are 30 km/h town roads. His traffic generator, so the car drives among a lorry, a police car and other cars. Night driving with high beam that dips for other cars. And his rule checker, so every run now measures speeding, leaving the road and near misses, not only collisions. A first check at night with 12 traffic cars passed all five scenarios with no rule violations.");
}
{
  const s = content("How Stanley steering fixes corner cutting");
  // left: pure pursuit
  card(s, 0.5, 1.1, 4.35, 3.85); card(s, 5.15, 1.1, 4.35, 3.85);
  txt(s, "Before: pure pursuit", { x: 0.7, y: 1.2, w: 4, h: 0.3, fontSize: 14, bold: true, color: C.accent3 });
  txt(s, "After: Stanley (Prithiv)", { x: 5.35, y: 1.2, w: 4, h: 0.3, fontSize: 14, bold: true, color: C.accent2 });
  function corner(ox) {
    s.addShape(pres.shapes.LINE, { x: ox + 0.4, y: 3.9, w: 2.0, h: 0, line: { color: C.text1, width: 3 }, objectName: "path" });
    s.addShape(pres.shapes.LINE, { x: ox + 2.4, y: 1.9, w: 0, h: 2.0, line: { color: C.text1, width: 3 }, objectName: "path" });
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: ox + 0.95, y: 3.72, w: 0.5, h: 0.36, rectRadius: 0.06, fill: { color: C.accent4 }, line: { color: C.accent4 }, objectName: "car" });
  }
  corner(0.7); corner(5.35);
  s.addShape(pres.shapes.LINE, { x: 0.7 + 1.45, y: 2.6, w: 0.95, h: 1.3, flipV: true, line: { color: C.accent3, width: 2, dashType: "dash" }, objectName: "chord" });
  s.addShape(pres.shapes.OVAL, { x: 0.7 + 2.32, y: 2.52, w: 0.16, h: 0.16, fill: { color: C.accent3 }, line: { color: C.accent3 }, objectName: "aim" });
  txt(s, "aims at a point 3–6 m ahead → the chord cuts the corner", { x: 0.7, y: 4.2, w: 3.95, h: 0.6, fontSize: 12 });
  s.addShape(pres.shapes.LINE, { x: 5.35 + 1.45, y: 3.55, w: 0, h: 0.35, line: { color: C.accent2, width: 2, endArrowType: "triangle" }, objectName: "error" });
  txt(s, "δ = ψ + atan(k·e/(v+1)) + atan(L·κ)", { x: 5.35, y: 1.6, w: 3.95, h: 0.3, fontSize: 12, fontFace: "Courier New", color: C.text1 });
  txt(s, "ψ: heading error\ne: offset at the front axle\nκ: how sharply the route bends", { x: 5.35, y: 2.05, w: 2.2, h: 1.0, fontSize: 11 });
  txt(s, "follows the path itself and pre-turns for the curve", { x: 5.35, y: 4.2, w: 3.95, h: 0.6, fontSize: 12 });
  s.addNotes("Why we switched. Pure pursuit steers towards a point a few metres ahead on the route. In a 90-degree turn the straight line to that point cuts the corner, which is why only 18 of 30 routes met our strict 1.5 m rule. Stanley, which Prithiv used, steers on the heading error plus the sideways error measured at the front axle, and a feed-forward term turns the wheels by the angle the curve itself needs. So the car follows the path instead of a shortcut.");
}
{
  const s = content("How the car handles traffic lights");
  const steps = [
    ["1", "Find stop lines", "Each light's stop line is placed on our planned route"],
    ["2", "Check the light", "Red: must stop. Amber: stop only if v²/(2·4 m/s²) fits"],
    ["3", "Treat as obstacle", "Stop line becomes a parked \"obstacle\"; overtaking blocked"],
    ["4", "Reuse stop logic", "Same braking distance and hold rules as for a parked car"],
    ["5", "Count violations", "Bumper over the line while red is logged per run"],
  ];
  steps.forEach((st, i) => {
    const x = 0.5 + i * 1.84;
    card(s, x, 1.15, 1.66, 2.75);
    s.addText(st[0], { shape: pres.shapes.OVAL, x: x + 0.15, y: 1.3, w: 0.45, h: 0.45, fill: { color: C.accent3 }, color: C.background1, fontSize: 14, bold: true, align: "center", valign: "middle", margin: 0, isTextBox: true, objectName: "step" });
    txt(s, st[1], { x: x + 0.15, y: 1.9, w: 1.4, h: 0.55, fontSize: 13, bold: true, color: C.text1 });
    txt(s, st[2], { x: x + 0.15, y: 2.5, w: 1.4, h: 1.35, fontSize: 11 });
  });
  txt(s, "Why this way: the stop-for-an-obstacle logic is already tested in 200+ runs, so a red light reuses it instead of adding a new controller.", { x: 0.5, y: 4.15, w: 9, h: 0.7, fontSize: 13 });
  s.addNotes("Traffic lights, the idea from Prithiv's signals node, built our way. We place every light's stop line on the planned route. At red the car must stop; at amber it stops only if it can do so comfortably. The stop line is then handed to the controller as a parked obstacle with the next lane closed, so the same braking logic we already tested in hundreds of runs does the stopping, but 4 m closer than for a parked car, so it now waits 1.6 to 3.6 m before the line. Every run also counts red-light crossings.");
}
{
  const s = content("Pedestrians, ambulances and the dashboard");
  const cards = [
    ["Pedestrian crosses", C.accent6, "A person steps onto the road as we approach.",
     ["LiDAR sees them; car brakes", "Stopped 10 m before them", "Never overtakes near pedestrians", "Drives on once the road is clear"]],
    ["Ambulance behind", C.accent4, "Blue lights come up from 45 m behind.",
     ["New YIELD mode (StVO §38)", "Right indicator, 0.7 m to the right", "Half speed until it has passed", "Then follows it at a safe gap"]],
    ["Instrument cluster", C.text1, "Prithiv's dashboard, now fed by our car.",
     ["Speedometer + speed limit", "Top-down LiDAR view", "Mode, indicators, distances", "Banners: red light, ambulance, pedestrian"]],
  ];
  cards.forEach((c, i) => {
    const x = 0.5 + i * 3.08;
    card(s, x, 1.1, 2.85, 3.5);
    s.addShape(pres.shapes.OVAL, { x: x + 0.2, y: 1.28, w: 0.32, h: 0.32, fill: { color: c[1] }, line: { color: c[1] }, objectName: "dot" });
    txt(s, c[0], { x: x + 0.62, y: 1.27, w: 2.15, h: 0.35, fontSize: 14, bold: true, color: C.text1, valign: "middle" });
    txt(s, c[2], { x: x + 0.2, y: 1.75, w: 2.5, h: 0.55, fontSize: 11, italic: true, color: C.accent5 });
    bullets(s, c[3], { x: x + 0.2, y: 2.35, w: 2.5, h: 2.15, fontSize: 12 });
  });
  txt(s, "First CARLA runs (Town03): pedestrian 1/1 and ambulance 1/1, no collisions. A full multi-town evaluation is next.", { x: 0.5, y: 4.75, w: 9, h: 0.35, fontSize: 11, italic: true, color: C.accent5 });
  s.addNotes("Three more things from Prithiv's work. First, pedestrians: when someone crosses, the LiDAR sees them, the car stops, about 10 m before them in our first run, and it is never allowed to overtake near a pedestrian. Second, ambulances: when one comes up behind with blue lights, the car switches to a new YIELD mode, indicates right, moves to the right edge of its lane and halves its speed until the ambulance has passed. Third, Prithiv's instrument cluster, now driven directly by our car: speed and limit, a top-down LiDAR view, what the car is doing, and warning banners. These are first runs only; the full evaluation comes next.");
}
{
  const s = content("What we did not take over, and why");
  const rows = [
    ["ROS 2 node layout and bridge", "Needs Ubuntu; asynchronous wall-clock timing would break our seeded, repeatable runs"],
    ["Siren sounds", "CARLA has no audio; the cluster banner and blue lights show it instead"],
    ["Fixed-time overtaking (25 s)", "Our LiDAR clearance check with closing speed is already tested on oncoming traffic"],
  ];
  rows.forEach((r, i) => {
    const y = 1.1 + i * 1.2;
    card(s, 0.5, y, 9.0, 0.84);
    tag(s, "NOT ADOPTED", 0.7, y + 0.28, C.accent5, C.background1, 1.15);
    txt(s, r[0], { x: 2.05, y: y + 0.12, w: 2.6, h: 0.6, fontSize: 13, bold: true, color: C.text1, valign: "middle" });
    txt(s, r[1], { x: 4.8, y: y + 0.12, w: 4.55, h: 0.6, fontSize: 12, valign: "middle" });
  });
  s.addNotes("We did not take everything. The ROS 2 node layout only runs on Ubuntu, and its wall-clock timing would break our repeatable runs. CARLA has no sound, so the sirens are shown as blue lights and a banner on the dashboard instead. And his time-based overtake limit is covered by our LiDAR clearance check, which is already tested with oncoming traffic.");
}

// ---------- 12. Testing ----------
section("Testing & results");
{
  const s = content("How we tested: 413 runs in CARLA");
  const hdr = (t) => ({ text: t, options: { bold: true, color: C.background1, fill: { color: C.text1 } } });
  const rows = [
    ["Run A", "Original system, perfect (ground-truth) perception", "90", "90/90"],
    ["Run B", "Final: LiDAR, 8 scenarios incl. curves and moving traffic, 6 towns", "200", "194/200"],
    ["Run C", "Camera steering + LiDAR", "18", "17/18"],
    ["Noise", "LiDAR with 5 cm range noise, Town04", "15", "15/15"],
    ["Run D", "City navigation through junctions", "60", "48/60*"],
    ["Regression", "Detector re-check after navigation changes", "30", "30/30"],
  ];
  s.addTable([[hdr("Test"), hdr("What it checks"), hdr("Runs"), hdr("Passed")], ...rows.map((r) => r.map((c, j) => ({ text: c, options: { bold: j === 0 || j === 3 } })))], {
    x: 0.5, y: 1.1, w: 9.0, colW: [1.2, 5.6, 0.9, 1.3], fontSize: 12, color: C.text2, border: { type: "solid", pt: 0.75, color: HEX.line }, rowH: 0.4, margin: 0.06, objectName: "test table",
  });
  txt(s, "Every run is seeded and repeatable. Each tick logs what the sensors reported next to the ground truth, so perception errors are measured, not guessed.\n* Strict route criterion; 29/30 routes reached the goal and all 30 obstacle stops passed.", { x: 0.5, y: 4.1, w: 9.0, h: 0.9, fontSize: 12 });
  s.addNotes("We ran six separate evaluations. Run A was the original system with perfect perception. Run B is the main result: LiDAR perception, eight scenarios, six towns. Run C uses the camera for steering. The noise test adds sensor noise. Run D is navigation. The regression run checks that the navigation changes did not break detection. Every run logs sensor output next to ground truth.");
}

// ---------- 13. Run B chart ----------
{
  const s = content("Final evaluation: 194/200 passed, 0 collisions");
  s.addChart(pres.charts.BAR, [{ name: "Success", labels: ["Empty lane", "Obstacle stop", "Obstacle avoid", "Curve lane", "Curve stop", "Lead vehicle", "Lead brake", "Oncoming overtake"], values: [100, 100, 100, 100, 100, 100, 84, 90] }], {
    x: 0.5, y: 1.05, w: 5.6, h: 3.95, barDir: "bar", chartColors: [HEX.green],
    showTitle: true, title: "Run B success rate by scenario (%)", titleFontSize: 12, titleColor: HEX.ink, titleFontFace: "+mn-lt",
    showValue: true, dataLabelPosition: "outEnd", dataLabelFontSize: 10, dataLabelColor: HEX.ink, dataLabelFontFace: "+mn-lt",
    valAxisHidden: true, valAxisMinVal: 0, valAxisMaxVal: 115, valGridLine: { style: "none" }, catGridLine: { style: "none" },
    catAxisLabelColor: HEX.gray, catAxisLabelFontSize: 10, catAxisLabelFontFace: "+mn-lt", catAxisOrientation: "maxMin", showLegend: false, objectName: "run B chart",
  });
  card(s, 6.4, 1.1, 3.1, 1.3);
  txt(s, "0", { x: 6.55, y: 1.18, w: 1.0, h: 0.65, fontSize: 36, bold: true, color: C.accent2, fontFace: "Arial" });
  txt(s, "collisions in 200 runs, 6 towns", { x: 6.55, y: 1.85, w: 2.8, h: 0.45, fontSize: 12 });
  txt(s, "The 6 that failed", { x: 6.4, y: 2.6, w: 3.1, h: 0.3, fontSize: 14, bold: true, color: C.text1 });
  bullets(s, [
    "4 lead-brake: stayed 5–6 m back but did not fully stop in time",
    "2 oncoming (Town02): manoeuvre not completed",
  ], { x: 6.4, y: 2.95, w: 3.1, h: 1.4, fontSize: 12 });
  txt(s, "Steering here uses map waypoints; LiDAR does all obstacle sensing.", { x: 6.4, y: 4.4, w: 3.1, h: 0.6, fontSize: 11, color: C.accent5, italic: true });
  s.addNotes("The final LiDAR evaluation: all core, curve and lead-vehicle scenarios passed. Four lead-brake runs kept a safe 5 to 6 m gap but did not come to a full stop within the time limit. Two oncoming-overtake runs in Town02 did not complete the manoeuvre. No run collided. Note that zero collisions does not mean every task finished; that is why we count the six failures.");
}

// ---------- Before / after Prithiv's ideas ----------
{
  const s = content("Navigation before and after Prithiv's ideas");
  const R = NEW_RESULTS;
  const val = (k) => (R && R[k] != null ? R[k] : "running");
  const hdr = (t) => ({ text: t, options: { bold: true, color: C.background1, fill: { color: C.text1 } } });
  const rows = [
    ["Routes within 1.5 m of the path", "18/30", val("stanley_strict"), val("lights_strict")],
    ["Routes reached the goal", "29/30", val("stanley_goal"), val("lights_goal")],
    ["Obstacle on route: stopped safely", "30/30", "–", val("lights_obstacle")],
    ["Red lights crossed", "not checked", "not checked", val("lights_violations")],
    ["Collisions", "0", val("stanley_collisions"), val("lights_collisions")],
  ];
  s.addTable([[hdr("Same 6 towns, same seeds"), hdr("Before: pure pursuit"), hdr("+ Stanley"), hdr("+ Stanley + lights")],
    ...rows.map((r) => r.map((c, j) => ({ text: c, options: { bold: j > 0, align: j > 0 ? "center" : "left" } })))], {
    x: 0.5, y: 1.1, w: 9.0, colW: [3.3, 1.9, 1.9, 1.9], fontSize: 13, color: C.text2, border: { type: "solid", pt: 0.75, color: HEX.line }, rowH: 0.5, margin: 0.07, objectName: "before after",
  });
  txt(s, R ? R.note : "CARLA evaluation still running; numbers are filled in when it finishes.", { x: 0.5, y: 4.35, w: 9, h: 0.6, fontSize: 12, italic: true, color: C.accent5 });
  s.addNotes(R ? R.speaker : "This slide compares navigation before and after Prithiv's ideas on exactly the same routes. The evaluation is still running.");
}

// ---------- 14. Fixes ----------
{
  const s = content("What broke in CARLA, and how we fixed it");
  const fixes = [
    ["Guardrails seen as obstacles on curves", "Corridor now follows the lane shape from the map"],
    ["Highway terrain read as an obstacle", "Road height estimated only from points near the path"],
    ["Braking late under heavy load", "Sensors read for the exact simulation frame"],
    ["Overtook a car that was starting to move", "Block overtaking as soon as motion is suspected"],
    ["Oncoming cars vanished at 60–80 m", "Keep last road height; longer-range LiDAR"],
    ["Route planner failed on 5 of 6 maps", "Skip one bad map segment instead of failing"],
  ];
  fixes.forEach((f, i) => {
    const col = i % 2, row = Math.floor(i / 2);
    const x = 0.5 + col * 4.6, y = 1.1 + row * 1.25;
    card(s, x, y, 4.4, 1.1);
    tag(s, "PROBLEM", x + 0.15, y + 0.12, C.accent3, C.background1, 0.85);
    txt(s, f[0], { x: x + 1.1, y: y + 0.1, w: 3.2, h: 0.35, fontSize: 12, bold: true, color: C.text1 });
    tag(s, "FIX", x + 0.15, y + 0.6, C.accent2, C.background1, 0.85);
    txt(s, f[1], { x: x + 1.1, y: y + 0.6, w: 3.2, h: 0.4, fontSize: 12 });
  });
  txt(s, "6 of 25 documented fixes. Each problem was found in a CARLA run and checked with a re-run.", { x: 0.5, y: 4.85, w: 9, h: 0.3, fontSize: 11, color: C.accent5, italic: true });
  s.addNotes("Most of our time went into problems that only appear in the simulator. A few examples: the LiDAR corridor swept onto guardrails on curves; terrain under an elevated highway looked like an obstacle; under load the sensors returned old frames so braking was late; the car once overtook a car that was just starting to move; oncoming cars disappeared from LiDAR at 60 to 80 m; and the route planner crashed on most maps because of one bad segment. The report lists all 25.");
}

// ---------- 15. Epic status ----------
section("Status & next steps");
{
  const s = content("Status per epic");
  const epics = [
    ["E1", "Simulation & tooling", "CARLA setup, repo, README, interfaces, config"],
    ["E2", "Lane detection", "Camera detector + lane-keeping; 91% held-out detection"],
    ["E3", "Obstacle detection", "LiDAR distance + adjacent-lane check; 0.3–0.5 m error"],
    ["E4", "Collision avoidance", "Stop / follow / avoid / abort; 130/130 static + curve runs"],
    ["E5", "Navigation & integration", "Junction routing, Stanley steering, traffic lights"],
    ["E6", "Management & docs", "Team plan, IEEE report, slides, demo video"],
  ];
  epics.forEach((e, i) => {
    const col = i % 2, row = Math.floor(i / 2);
    const x = 0.5 + col * 4.6, y = 1.1 + row * 1.3;
    card(s, x, y, 4.4, 1.15);
    s.addText(e[0], { shape: pres.shapes.OVAL, x: x + 0.15, y: y + 0.2, w: 0.6, h: 0.6, fill: { color: C.text1 }, color: C.accent1, fontSize: 14, bold: true, align: "center", valign: "middle", margin: 0, isTextBox: true, objectName: "epic" });
    txt(s, e[1], { x: x + 0.95, y: y + 0.12, w: 2.3, h: 0.35, fontSize: 14, bold: true, color: C.text1 });
    tag(s, "DONE", x + 3.45, y + 0.14, C.accent2, C.background1, 0.8);
    txt(s, e[2], { x: x + 0.95, y: y + 0.5, w: 3.3, h: 0.6, fontSize: 12 });
  });
  txt(s, "Open item: Section IV (Innovation) drafted, waiting for Khalid's review.", { x: 0.5, y: 4.95, w: 9, h: 0.25, fontSize: 11, color: C.accent5, italic: true });
  s.addNotes("Every epic on the board is in Done with evidence attached. The one open item is the review of the innovation section of the report.");
}

// ---------- 16. Limitations ----------
{
  const s = content("Limitations and what comes next");
  card(s, 0.5, 1.1, 4.3, 3.0);
  txt(s, "What the system cannot do yet", { x: 0.7, y: 1.25, w: 3.9, h: 0.3, fontSize: 15, bold: true, color: C.accent3 });
  bullets(s, [
    "No pedestrians or dense traffic tested",
    "Steering and LiDAR corridor still use map geometry",
    "Camera weak in Town01 and Town10HD",
    "No right-of-way at junctions without lights",
    "Pedestrian and ambulance scenarios: first runs only",
  ], { x: 0.7, y: 1.7, w: 3.9, h: 3.1, fontSize: 13 });
  card(s, 5.2, 1.1, 4.3, 3.0, C.text1);
  txt(s, "Next steps (to 24 Nov review)", { x: 5.4, y: 1.25, w: 3.9, h: 0.3, fontSize: 15, bold: true, color: C.accent1 });
  bullets(s, [
    "Firmer low-speed stopping behind a braking car",
    "Finish oncoming overtakes reliably",
    "Fix camera edge and heading errors per town",
    "Test camera and noise with moving traffic and weather",
    "Full evaluation of pedestrian + ambulance scenarios; right-of-way",
  ], { x: 5.4, y: 1.7, w: 3.9, h: 3.1, fontSize: 13, color: C.background1 });
  s.addNotes("Honest limitations: we have not tested pedestrians or dense traffic; steering in the main evaluation and the LiDAR corridor still rely on the map; the camera is weak in two towns; junctions without lights have no right-of-way logic; and we have no emergency-vehicle scenario yet, which is where Prithiv's yielding rule would come in next. These are the next steps.");
}

// ---------- 17. Close ----------
{
  const s = pres.addSlide({ masterName: "DARK_TITLE", sectionTitle: sec });
  s.addText("Thank you — questions?", { placeholder: "title" });
  s.addText("Run B: 194/200, 0 collisions · Camera: 17/18 · Navigation: 30/30 obstacle stops\nCode, results and report in the group repository; demo video on its Releases page.", { placeholder: "body" });
  for (let i = 0; i < 6; i++) s.addShape(pres.shapes.RECTANGLE, { x: 0.6 + i * 1.55, y: 4.95, w: 0.9, h: 0.08, fill: { color: C.accent1 }, line: { color: C.accent1 }, objectName: "lane dash" });
  s.addNotes("To sum up: the car follows its lane, detects obstacles with LiDAR, stops, follows or overtakes safely, and navigates through junctions. We are happy to take questions or show the demo video.");
}

(async () => {
  await pres.writeFile({ fileName: OUT });
  await applyTheme(OUT, THEME);
  console.log("wrote " + OUT);
})();
