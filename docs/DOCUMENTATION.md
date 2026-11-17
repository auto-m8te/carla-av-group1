# Group 1 — Autonomous Driving in CARLA: Project Documentation

Engineering Teamwork, WS2026/27, SRH Berlin (Prof. A. Laboudi)

Team: Khalid · Abir · Adil · Prithiv · Omar

Status: 17 November 2026 (Sprint 1: 10–24 November 2026)

This document explains, step by step, what the team built, why each choice was made,
how it was tested, and how to run it. It was written with Claude Code from the
repository, its results files and the team plan.

---

## 1. The task

Build, as a team, a car in the CARLA 0.9.16 simulator that can:

1. **follow its lane** on straight roads and curves,
2. **detect obstacles** (parked cars, slow or braking cars, oncoming traffic, pedestrians),
3. **avoid collisions** by either stopping or steering around the obstacle,
4. **navigate** a city: plan a route and drive it through junctions safely.

All four work in simulation today. Sections 4–9 explain how, Section 10 how well.

---

## 2. Team and way of working

| Member | Role on the Trello board | Epics |
|---|---|---|
| Prithiv | Simulation & integration, board owner; built the ROS 2 "Lanebot" | E1, E5 |
| Adil | Lane detection & steering; literature review | E2 |
| Abir | Obstacle detection (sensors); methodology | E3 |
| Omar | Collision avoidance, testing, documentation, repository | E4, E6 |
| Khalid | City navigation & innovation (joined after kick-off) | E5 |

Epics: **E1** simulation & tooling · **E2** lane detection · **E3** obstacle detection ·
**E4** collision avoidance · **E5** navigation & integration · **E6** management & docs.

| Tool | Rule |
|---|---|
| Trello | one card per task; a card is *Done* only with evidence (CSV, plot, video) |
| GitHub | branch + pull request per card; tests must pass; videos go to Releases |
| WhatsApp | blockers posted the same day |
| Stand-ups | done · next · blocked; sprint review on 24 Nov |

Roles were a starting point: when a card was blocked, someone else picked it up so the whole
system could be integrated and tested (see `docs/team_plan.md`, Section 6).

---

## 3. Development, step by step

| Step | Date | What was added | Commit |
|---|---|---|---|
| 1 | 10 Nov | Base car: lane following + LiDAR obstacle stop / avoid | `a52cc28` |
| 2 | 10 Nov | LiDAR check that the neighbouring lane is free | `d3986ae` |
| 3 | 10 Nov | Fix LiDAR false alarms past road turns; safe CARLA teardown; resumable runs | `ac8be6a` |
| 4 | 10 Nov | First 90-run LiDAR evaluation (Run A) | `fc61f79` |
| 5 | 10 Nov | Sharp curves, moving lead car, oncoming traffic | `aa2018f` |
| 6 | 10 Nov | Camera lane detection + lane-keeping controller | `befdb7c` |
| 7 | 10 Nov | Camera detector tuned on 1,674 labelled CARLA frames | `f2422be` |
| 8 | 10 Nov | Never overtake something that may be moving; oncoming safety metric | `45d99d5` |
| 9 | 10 Nov | Final 200-run LiDAR evaluation (Run B) | `601aa3d` |
| 10 | 10 Nov | Frame-synchronised sensor reads; camera + LiDAR closed loop (Run C) | `d3de8e7` |
| 11 | 10 Nov | LiDAR noise test (15/15) | `6c337d6` |
| 12 | 11 Nov | City navigation through junctions (Run D) | `97a412f` |
| 13 | 11 Nov | Route planner robust to bad map segments; speed-dependent look-ahead | `9c570e1`, `e3435f7` |
| 14 | 11 Nov | IEEE report, slides, Section IV innovation | `65ed70b` |
| 15 | 17 Nov | Prithiv's Lanebot reviewed and its ideas integrated (Section 9) | `7cf6e88` |

Every step was merged only after it ran in CARLA and its automated tests passed.

---

## 4. Architecture

```
CARLA 0.9.16 (synchronous, 20 Hz)
   |                     |
RGB camera           roof LiDAR
   |                     |
lane module          obstacle module ---- adjacent-lane check
   |  steering_angle     |  obstacle_distance, adjacent_lane_clear
   +----------+----------+
              v
   decision: stop-or-avoid state machine  (+ route, traffic lights, rules)
              v
   control: steering, throttle, brake  ->  CARLA vehicle, every 0.05 s
```

The modules share three small values, so team members could build them in parallel:
`steering_angle`, `obstacle_distance`, `adjacent_lane_clear`.
Plug-in point: `CarlaAvoidanceBridge` in `carla_av/avoidance/carla_adapter.py`.

**Why CARLA's Python API directly (not ROS 2)?** Synchronous mode makes every run seeded and
repeatable, it runs on the team's Windows machine, and 277 unit tests run without a simulator.

---

## 5. Obstacle detection with LiDAR (E3)

Code: `carla_av/perception/obstacle_detection.py`, `lidar_sensor.py`

1. **Range gate:** keep points from the front bumper to 60 m.
2. **Corridor from the map:** keep only points inside a corridor shaped like our lane (or
   route), car width + 0.3 m each side.
3. **Road removal:** estimate the road height in 2 m bins from points next to the path.
4. **Cluster check:** an obstacle needs at least 3 points within 1 m and must stand at least
   0.25 m tall.
5. Output: `obstacle_distance` (metres to the nearest obstacle in our path, or none).

The same filter over the neighbouring lane answers "is it free for the whole manoeuvre?":
`distance − closing speed × 9 s > 10 m`, continuously for 1 s.

**Why a map-shaped corridor instead of a box?** A fixed box swept onto guardrails and
buildings in curves. Following the lane cut false alarms in the first 90-run test from
1,564 ticks to 136. Distance error against ground truth: 0.3–0.5 m.

---

## 6. Decision and control (E4)

Code: `carla_av/avoidance/collision_avoidance.py` (pure Python, unit-tested)

| Mode | When |
|---|---|
| CRUISE | drive the lane at target speed |
| FOLLOW | moving car ahead: keep 6 m + 1.8 s × speed |
| BRAKE → STOPPED | obstacle ahead and the next lane is blocked |
| AVOID → PASSING → RETURN | obstacle ahead and the next lane is free: overtake and come back |
| ABORT | next lane closes mid-manoeuvre: go back behind the obstacle |
| EMERGENCY_STOP | anything closer than 5 m, in any mode |
| YIELD | emergency vehicle near: right edge of the lane, half speed (from Prithiv) |

Braking point: `d_stop = v·t_react + v²/(2a) + margin`, so faster cars react earlier.
Steering: PD on lateral offset and heading; speed: feed-forward + P controller.

**Why a rule-based state machine?** Every decision traces to one rule, no training data is
needed, and each rule is unit-tested. **Why speed-dependent braking?** A fixed trigger
distance was too short at speed (Town01 bug).

---

## 7. Lane detection with a camera (E2)

Code: `carla_av/perception/lane_detection.py`, `camera_sensor.py`

1. White, yellow and curb masks (local contrast) → edges → Hough segments in a wide ROI.
2. Pick a left/right boundary pair that matches the expected lane width.
3. Project to the road, estimate lateral offset and heading.
4. PD steering with a median filter and a rate limit.

Tuned on drives 0–1 of 1,674 automatically labelled frames, tested on held-out drive 2
(558 frames): **91.2 % detection, 0.27 m offset error**. Closed loop with LiDAR (Run C):
**17/18**. Still weak in Town01 and Town10HD.

---

## 8. City navigation (E5)

Code: `carla_av/navigation/route_planner.py`, `route_follower.py`, `traffic_lights.py`

1. **Plan:** shortest path (Dijkstra) on the map's road graph.
2. **Follow:** Stanley steering at the front axle + curve feed-forward
   (`δ = ψ + atan(k·e/(v+1)) + atan(L·κ)`), adopted from Prithiv. The old pure pursuit aimed at
   a point ahead and cut corners at junction exits.
3. **Slow down:** 5 m/s near junctions, 4 m/s in sharp turns; no overtaking near junctions.
4. **Traffic lights:** every light's stop line is placed on the route. Red: stop. Amber: stop
   only if `v²/(2·4 m/s²)` fits. The stop line is handed to the controller as a parked
   obstacle, so the tested stop logic does the braking; the car waits 1.6–3.6 m before the line.
5. **Safety layer:** the LiDAR corridor follows the route, so the same detection and stop
   logic protects the whole route.

---

## 9. Integrating Prithiv's Lanebot (17 Nov)

Prithiv built a separate version of the car: ROS 2 Jazzy on Ubuntu, driving the Town04
motorway with German traffic rules (StVO). It is in `prithiv/`. It cannot run on the team's
Windows CARLA machine, so its **ideas** were ported into our pipeline, each with unit tests.

### 9.1 What was taken, and how

| Prithiv's idea (file) | In our car | Why |
|---|---|---|
| Stanley steering + curve feed-forward (`control_node`) | route follower, default | stop corner cutting |
| Stop at red / amber lights, StVO §37 (`signals_node`) | `navigation/traffic_lights.py` | routes used to run red lights |
| Left lane must continue (`left_room`) | target-lane check along the manoeuvre | never pull into a lane that ends |
| Broken-line rule, StVO §5 | `--obey-markings` (optional) | legal overtakes |
| Indicators before lane changes | blinkers in AVOID / RETURN / YIELD | shows intent |
| Make way for emergency vehicles, StVO §38 | new `YIELD` mode | ambulances get through |
| Pedestrians | LiDAR stops the car; no overtaking near them | protect people |
| Instrument cluster (`cluster_node`) | `--cluster` dashboard window | shows what the car sees/decides |
| Speed limits at 90 %, slow before signs, "Fahren auf Sicht" | `avoidance/traffic_rules.py` | 9 of 200 old runs exceeded 33 km/h in 30 zones |
| Mixed traffic (`traffic_node`) | `--traffic N` (Traffic Manager: lorry, police, cars) | test among other road users |
| Night + headlight dipping | `--night` | realistic night driving |
| Rule checker (`tools/monitor.py`) | `scenarios/rule_monitor.py` + analysis table | measure legal driving, not only crashes |

### 9.2 What was not taken, and why

| Not taken | Why |
|---|---|
| ROS 2 node layout and bridge patches | Ubuntu only; wall-clock timing breaks seeded, repeatable runs |
| Merging 3 LiDAR sweeps | smears moving cars over 3 frames; our detector reads the exact current frame |
| Siren / indicator sounds | CARLA has no audio; blue lights and cluster banners show it |
| Fixed-time overtaking limit | our LiDAR clearance check with closing speed is already tested |
| Gazebo draft | marked unused by Prithiv |

### 9.3 New scenarios

- `pedestrian_crossing`: a pedestrian crosses 35–55 m ahead when we are 28 m away.
  First run: stopped 10 m before them, drove on when clear.
- `emergency_vehicle`: an ambulance with blue lights starts 45 m behind at 11 m/s and passes.
  First run: car yielded (0.7 m right, half speed), then followed it.

---

## 10. Testing and results

Every run is seeded and logs, each tick, what the sensors reported next to the ground truth,
so perception errors are measured, not guessed.

| Test | What it checks | Runs | Passed |
|---|---|---|---|
| Run A | original system, ground-truth perception | 90 | 90/90 |
| Run B (final) | LiDAR, 8 scenarios incl. curves and moving traffic, 6 towns | 200 | 194/200, 0 collisions |
| Run C | camera steering + LiDAR | 18 | 17/18 |
| Noise | LiDAR with 5 cm range noise, Town04 | 15 | 15/15 |
| Run D | city navigation (pure pursuit, before Prithiv's ideas) | 60 | obstacle stops 30/30, goal reached 29/30, strict route 18/30, 0 collisions |
| Regression | detector re-check after navigation changes | 30 | 30/30 |
| Check run (17 Nov) | Town03 at night, 12 traffic cars, incl. pedestrian + ambulance | 5 | 5/5, 0 collisions, 0 s speeding, 0 s off-road, 0 near misses |

Run B failures: 4 lead-brake runs stayed 5–6 m back but did not fully stop in time;
2 oncoming overtakes in Town02 did not complete. 277 unit tests run without CARLA
(`python -m pytest -q`).

**Pending:** re-running Run D with Stanley steering and traffic lights (same seeds) to measure
the before/after effect of Prithiv's ideas.

---

## 11. Problems found in CARLA and how they were fixed (selection)

| Problem | Fix |
|---|---|
| Guardrails seen as obstacles on curves | corridor follows the lane shape from the map |
| Terrain under an elevated highway read as an obstacle | road height only from points near the path |
| Braking late under load (stale sensor frames) | read the sensor for the exact simulation frame |
| Overtook a car that was starting to move | block overtaking as soon as motion is suspected |
| Oncoming cars vanished at 60–80 m | keep last road height; 64-channel 185 m LiDAR for moving traffic |
| Route planner failed on 5 of 6 maps | skip one unsampleable segment instead of failing |
| CARLA crashes when switching maps after many runs | restart CARLA fresh per town, then load the map once |

All 25 development fixes are listed in the IEEE report (`report/`, Section V).

---

## 12. How to run it

**Setup (team machine):** CARLA 0.9.16 in `C:\CARLA`, Python 3.11 venv in `carla_av/.venv`
(`uv venv --python 3.11 .venv && uv pip install -r requirements.txt`).

> Windows App Control currently blocks `.venv\Scripts\python.exe`. Use the uv Python the venv
> was built from with the venv's packages: `demo.ps1` does this automatically.

**Live demo** (starts CARLA if needed, opens the instrument cluster):

```powershell
cd carla_av
.\demo.ps1                          # parked car: stop, then overtake (Town04)
.\demo.ps1 city -Traffic 12 -Night  # pedestrian + ambulance, with traffic, at night (Town03)
.\demo.ps1 navigation               # route through junctions, stops at red lights (Town01)
.\demo.ps1 moving                   # car ahead, braking car, oncoming traffic
.\demo.ps1 curves                   # sharp curves
```

**Evaluation runs and analysis:**

```powershell
python -m scenarios.run_scenarios --scenario everything --runs 5 --obstacle lidar --adjacent lidar --out results_x
python -m analysis.analyze_results --results results_x     # summary.md + plots
python -m pytest -q                                         # unit tests, no CARLA
```

Useful flags: `--traffic N`, `--night`, `--cluster`, `--realtime --hud`, `--lane camera`,
`--route-steer pursuit`, `--ignore-lights`, `--ignore-limits`, `--obey-markings`.

---

## 13. Repository map

| Path | Contents |
|---|---|
| `carla_av/avoidance/` | decision logic, CARLA bridge, speed rules |
| `carla_av/perception/` | LiDAR detector, camera lane detector, tracking |
| `carla_av/navigation/` | route planner, Stanley route follower, traffic lights |
| `carla_av/scenarios/` | scenario runner, traffic, rule checker, instrument cluster, video |
| `carla_av/analysis/` | metrics, tables, plots |
| `carla_av/tests/` | 277 unit tests |
| `carla_av/results*/` | CSV logs and summaries of the evaluations |
| `prithiv/` | Prithiv's ROS 2 Lanebot (reference) |
| `report/` | IEEE report, slides (`Group1_Progress_Presentation.pptx`), figures, videos |
| `docs/` | team plan, this documentation |

---

## 14. Limitations and next steps

- No dense traffic or crowds of pedestrians evaluated yet; pedestrian and ambulance
  scenarios have first runs only.
- Steering in the main evaluation and the LiDAR corridor still use map geometry.
- Camera lane detection is weak in Town01 and Town10HD.
- Junctions without traffic lights have no right-of-way logic.
- Next: full multi-town evaluation of the new scenarios and of Run D with Stanley + lights;
  right-of-way at unsignalled junctions; camera fixes per town.
