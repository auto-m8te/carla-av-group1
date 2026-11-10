# Report sections — Omar (draft)

> Section V uses real CARLA results (carla_av/results/: 90 runs, 6 towns).
> The remaining [[...]] placeholders are teammates' parts.

---

## I. Introduction  *(shared: Adil + Abir + Omar)*

Road traffic crashes kill about 1.19 million people every year, and most are linked to
human error [1]. Automated driving aims to reduce this by giving a vehicle the ability to
perceive its environment, decide, and act without a human driver [2]. Even the most basic
automated-driving function has to solve three problems at the same time: staying inside a
lane, noticing objects in the way, and choosing a safe reaction.

Testing these functions on real roads is expensive and risky. Simulators such as CARLA [3]
offer realistic urban environments, configurable sensors, and repeatable traffic scenarios,
so a control strategy can be developed and validated safely before any hardware is involved.

**Problem statement.** The goal of this project is to develop, as a team, a simulated
autonomous vehicle in a CARLA city environment that can (1) follow a driving lane,
(2) detect obstacles, (3) avoid collisions by either stopping or steering around, and
(4) navigate safely.

**Approach in brief.** The system has three layers: perception (camera-based lane
detection with OpenCV, plus LiDAR/depth-based obstacle detection), decision (a
stop-vs-steer-around state machine), and control (PID steering with throttle and brake
commands). The modules share small, agreed interfaces (`steering_angle`,
`obstacle_distance`), which allowed team members to develop them in parallel.

**Contributions.** [[Khalid's Innovation section summarises these — keep consistent]]
- A modular perception → decision → control pipeline in CARLA.
- A deterministic collision-avoidance state machine that uses a speed-dependent stopping
  distance and checks whether the adjacent lane is free before choosing to steer around.
- A repeatable three-scenario evaluation (empty lane, obstacle-stop, obstacle-avoid) with
  quantitative metrics.

**Report structure.** Section II reviews related work, Section III describes the
methodology, Section IV presents innovations, Section V reports findings and results, and
Section VI concludes.

---

## V. Findings & Results Analysis  *(Owner: Omar)*

### A. Test setup

All experiments were run in CARLA 0.9.16 [3] in synchronous mode with a fixed time step
of 0.05 s (20 Hz), on a desktop PC with an NVIDIA RTX 3060 GPU. To check that the system
is not tuned to a single map, every scenario was run in six CARLA towns: Town01, Town02,
Town03, Town04, Town05 and Town10HD. These cover small single-lane towns, an urban grid
with curves, a multi-lane highway, and a dense downtown map.

Each scenario was repeated five times per town (6 towns × 3 scenarios × 5 runs = **90 runs**).
Every run was randomised with a fixed seed, so the results can be reproduced exactly:

- **Start location**: chosen at random from road stretches with an adjacent driving lane,
  no junction, and at most 25° of heading change (mild curves are allowed).
- **Obstacle distance**: 35–55 m ahead of the start point.
- **Cruise speed**: 6–10 m/s (≈ 22–36 km/h, typical urban speeds).
- **Lane width**: read from the map for each run (4.0 m on Town01/02, 3.5 m elsewhere).

| Scenario | Setup | Expected behaviour |
|---|---|---|
| S1 Empty lane | No obstacles, drive 100 m | Follow the lane, no braking |
| S2 Obstacle-stop | Parked car ahead, adjacent lane blocked by a second parked car | Brake and stop with a gap of at least 2 m |
| S3 Obstacle-avoid | Parked car ahead, adjacent lane free | Change lane, pass, return to the original lane |

On Town01, Town02 and Town10HD the chosen roads have only one lane per direction, so the
vehicle overtook through the (empty) oncoming lane. This applied to 15 of the 30 S3 runs.
On the other towns it used a second lane in the same direction.

Every tick was logged to CSV (speed, controller mode, obstacle distance, lateral offset
from the original lane centre, and control commands). A collision sensor on the ego
vehicle recorded any contact.

**Perception sources.** The experiment was run twice with the same 90 seeds:

- **Run A (ground truth):** obstacle distance and "is the adjacent lane free" are computed
  geometrically from actor positions. This isolates the decision and control logic.
- **Run B (LiDAR):** both values come from the team's LiDAR obstacle module (a 32-channel
  roof LiDAR, one full sweep per tick; see Section III). Ground truth is still logged next to
  the LiDAR output each tick, so detection errors can be measured.

In both runs lane steering is pure pursuit on map waypoints. [[Rerun with Adil's OpenCV
lane module once integrated.]] In Run B, Town02 and Town03 were loaded as their layered
`_Opt` variants (same OpenDRIVE road network, same seeds), because the standard versions
crashed CARLA's renderer on our machine ("Shader compilation failures are Fatal").

### B. Metrics

- **Success rate**: the share of runs that met the expected behaviour with no collision.
  - S1: the vehicle never left the lane (|offset| < 1.0 m).
  - S2: the vehicle stopped with at least a 2 m gap.
  - S3: the vehicle completed AVOID → PASSING → RETURN and ended within 0.5 m of the
    original lane centre.
- **Lane-keeping error**: RMS and maximum lateral offset from the lane centre while in
  CRUISE mode.
- **Reaction distance / time**: the distance to the obstacle when the controller first
  left CRUISE, and the time between first detection and that reaction.
- **Minimum gap**: the closest bumper-to-bumper distance to the obstacle in the path.

### C. Results

**Table I – Overall results (30 runs per scenario; values are mean (min–max))**

| Scenario | Runs | Success rate | Collisions | Lane RMS err (m) | Lane max err (m) | Reaction dist (m) | Reaction time (s) | Min gap (m) |
|---|---|---|---|---|---|---|---|---|
| S1 empty_lane | 30 | 30/30 (100%) | 0 | 0.02 (0.00–0.12) | 0.10 (0.00–0.58) | – | – | – |
| S2 obstacle_stop | 30 | 30/30 (100%) | 0 | 0.03 (0.00–0.20) | 0.05 (0.00–0.34) | 10.62 (8.75–11.75) | 6.46 (4.90–9.75) | 6.88 (5.75–9.75) |
| S3 obstacle_avoid | 30 | 30/30 (100%) | 0 | 0.04 (0.03–0.05) | 0.30 (0.28–0.30) | 26.38 (23.75–30.75) | 3.43 (1.95–6.10) | 12.32 (8.75–17.75) |

**Table II – Success per town (5 runs each)**

| Town | S1 empty_lane | S2 obstacle_stop | S3 obstacle_avoid | Collisions |
|---|---|---|---|---|
| Town01 | 5/5 | 5/5 | 5/5 | 0 |
| Town02 | 5/5 | 5/5 | 5/5 | 0 |
| Town03 | 5/5 | 5/5 | 5/5 | 0 |
| Town04 | 5/5 | 5/5 | 5/5 | 0 |
| Town05 | 5/5 | 5/5 | 5/5 | 0 |
| Town10HD | 5/5 | 5/5 | 5/5 | 0 |

**Table III – Run B: LiDAR perception (same 90 seeds)**

| Scenario | Runs | Success rate | Collisions | Reaction dist (m) | Min gap (m) | LiDAR distance error (MAE, m) |
|---|---|---|---|---|---|---|
| S1 empty_lane | 30 | 30/30 (100%) | 0 | – | – | – |
| S2 obstacle_stop | 30 | 30/30 (100%) | 0 | 11.05 (9.51–12.53) | 7.02 (5.41–10.32) | 0.48 (0.21–1.97) |
| S3 obstacle_avoid | 30 | 30/30 (100%) | 0 | 26.85 (23.95–33.18) | 13.78 (10.19–19.19) | 0.44 (0.27–1.76) |

All six towns: 15/15 runs each, 0 collisions. Of 23,715 logged ticks, the LiDAR
reported an obstacle that ground truth did not in 136 ticks (0.6%). 130 of these came from
three Town05 runs, where a steep road crest 33–53 m ahead was briefly read as an obstacle
while the car pulled away (semantic-LiDAR tag: road). None of them changed the vehicle's
behaviour. "Missed" ticks (373) are almost all the first tick of a run, before the first
sweep arrives, and the moment the obstacle slides out of the corridor during a lane change.

**Additional observations from the logs**

- S2: the stopping gap was 5.75–9.75 m in every town. This is always above the 4 m design
  margin, and none of the 30 runs needed the emergency brake (< 5 m).
- S3: no run needed an emergency stop. The peak lateral displacement was 3.21–3.72 m,
  which is one full lane. The whole manoeuvre (lane change → pass → return) took 10.4 s on
  average (7.7–19.6 s).

*Fig. 1 – `figures/Town01_obstacle_avoid_run1.png`: overtaking through the oncoming lane
on Town01. Blue = AVOID/PASSING, green = RETURN.*
*Fig. 2 – `figures/Town03_obstacle_stop_run1.png`: braking (orange) and holding stopped
(red) about 10 m from the obstacle.*
*Fig. 3 – `figures/Town04_obstacle_avoid_run2.png`: lane change on the multi-lane
highway.*
*Fig. 4 – `figures/Town05_empty_lane_run1.png`: lane following with no obstacle.*

### D. Discussion

**What worked.**

- **Robust across maps.** All 90 runs succeeded with zero collisions across six towns,
  with different lane widths, mild curves, both lane-change directions (same-direction
  lane and oncoming lane), and randomised speeds and distances. The logic does not depend
  on one specific map.
- **Stopping is speed-aware.** The braking point is the stopping distance
  d = v·t_react + v²/(2a) + margin, so faster runs started braking earlier. Reaction
  distances of 8.75–11.75 m match the formula for 6–10 m/s. The vehicle always stopped
  smoothly before reaching the emergency zone.
- **Lane change starts early enough.** The avoid trigger adds the distance covered during
  a lane change (v × 2.5 s), so S3 always began its manoeuvre 24–31 m ahead and kept at
  least 8.75 m from the obstacle.
- **Stop vs. steer-around decision.** In S2 the adjacent lane was correctly judged
  blocked in 30/30 runs, so the vehicle never tried to pull out into the second parked
  car.

**Trade-off between stopping and avoiding.** Avoiding keeps the vehicle moving, but it
needs a free adjacent lane and takes about 10 s and roughly 60–100 m of road. Stopping is
simpler and always available, but it blocks progress indefinitely. Our policy prefers
avoidance only when the adjacent lane is verified free over the whole detection range
(60 m). Otherwise it stops.

**What did not work at first (development iterations).** Testing in CARLA exposed
problems that unit tests on a simplified model did not catch. Each was fixed and
regression-tested (28 automated tests):

| # | Problem observed in CARLA | Cause | Fix |
|---|---|---|---|
| 1 | Collision in S2 on Town04: drove into the blocked lane | Lane adjacency compared CARLA road/lane IDs, which change between road segments | Geometric lateral-offset check |
| 2 | Collision while passing | Emergency braking disabled in PASSING mode | Brake for any obstacle ahead (gap ≥ 0) in every mode |
| 3 | Runs ended instantly | Odometer started at 544 m (actor at origin before first tick) | Initialise the odometer after the first tick |
| 4 | Lane change overshot by ~1.9 m | P-only lateral control ignores heading | PD-style control with a heading-error term |
| 5 | Offset jumped by one lane mid-manoeuvre | Original lane lost when the road segment changed | Track the home lane by walking its waypoints forward |
| 6 | Car accelerated without limit | Constant throttle | Feed-forward + P speed controller |
| 7 | Town01: emergency stop mid-lane-change, then steered back toward the obstacle | Fixed trigger distance too short; emergency stop reset the avoid state | Speed-scaled trigger; keep the manoeuvre state through braking |
| 8 | Collision in S2 on Town03 (1 of 18 runs) | Adjacent-lane check only covered 30 m, so the blocker was out of range | Check the full 60 m detection range; brake if the target lane becomes blocked after committing |
| 9 | 4/30 S2 runs briefly started a lane change before aborting (safe, but up to 1.8 m into the lane) | On curves the blocker was measured against the lane direction at the ego's position | Measure each actor against the lane at its own position; false starts went from 4/30 to 0/30 |
| 10 | LiDAR: 101 false-alarm ticks in one Town04 avoid run; gap *grew* while driving | The search corridor bent with the yaw rate and swept onto the guardrail | Corridor follows the lane shape from the map |
| 11 | LiDAR: car changed lanes on an empty Town04 road | Elevated highway: terrain below the road pulled the road-height estimate down, so the road looked like an obstacle | Estimate road height only from points near the path |
| 12 | LiDAR: 1,564 false-alarm ticks in the first 90-run LiDAR evaluation, mostly Town01/02/10HD | Where the road turns, the corridor continued straight ahead into trees and buildings (found with semantic-LiDAR tags) | Only search as far as the lane path reaches; false alarms fell to 136 |

**Limitations.**

1. **Perception is partly idealised.** Run B uses real LiDAR for obstacles, but the LiDAR
   is noise-free (CARLA ray casting), the lane corridor comes from the map, and lane
   steering is still ground truth. Sensor noise and the camera lane module are untested.
   Steep road crests can still be read as obstacles (Town05).
2. **Static obstacles only.** Parked cars were used. Moving obstacles, pedestrians, and
   oncoming traffic during an overtake were not tested. Overtaking through the oncoming
   lane currently assumes the 60 m range is enough, while real oncoming traffic needs a
   time-to-collision check.
3. **Distance resolution.** The fallback obstacle detector walks the lane in 1 m steps,
   so the reported gaps and reaction distances are accurate to about ±1 m (visible as
   steps in the plots).
4. **Lane keeping in curves.** Two S1 runs (Town02 run 2, Town10HD run 3) reached
   0.54–0.58 m offset in curves because the pure-pursuit fallback cuts corners. This is
   well inside the 3.5–4 m lane, but it is the largest lane-keeping error observed.
5. **Slow manoeuvre.** Lane changes are deliberately gentle (steer ≤ 0.4), and the return
   phase converges slowly. A planned trajectory would shorten the 10 s manoeuvre.
6. **Sample size.** 5 runs per town and scenario (90 in total). CARLA in synchronous mode
   is deterministic, so variation comes only from the randomised start, distance and speed.

---

## VI. Conclusion & Future Work  *(shared: Adil + Abir + Omar)*

This project produced a simulated autonomous vehicle in CARLA that follows its lane,
detects obstacles ahead, and decides whether to stop or steer around them. The modular
design, with separate lane-following, obstacle-detection, and collision-avoidance modules
connected through small interfaces, let the team develop and test each part on its own
before integrating them. Across 90 randomised test runs in six CARLA towns, the vehicle succeeded in 100% of
the empty-lane, obstacle-stop and obstacle-avoid scenarios with zero collisions, using
ground-truth perception, and again in 90/90 runs with zero collisions when obstacles and the
adjacent lane were detected by the team's LiDAR module. Integrating the camera lane module is
the next step.

Future work:
1. **Dynamic obstacles**: estimate obstacle velocity and use time-to-collision instead of
   distance alone.
2. **Sensor fusion**: combine camera object detection (for example a YOLO-family
   detector) with LiDAR to reduce false positives.
3. **Smoother manoeuvres**: replace the lateral P-controller with a planned trajectory
   (for example a polynomial lane-change path [8]) or model predictive control.
4. **Learning-based control**: compare the rule-based pipeline against end-to-end or
   imitation-learning approaches [6], [7].
5. **Broader testing**: more towns, weather and lighting conditions, junctions, and
   CARLA ScenarioRunner scenarios.

---

## References (IEEE)

[1] World Health Organization, *Global Status Report on Road Safety 2023*. Geneva, Switzerland: WHO, 2023.

[2] SAE International, "Taxonomy and definitions for terms related to driving automation systems for on-road motor vehicles," SAE Standard J3016, 2021.

[3] A. Dosovitskiy, G. Ros, F. Codevilla, A. López, and V. Koltun, "CARLA: An open urban driving simulator," in *Proc. 1st Annu. Conf. Robot Learning (CoRL)*, 2017, pp. 1–16.

[4] J. Canny, "A computational approach to edge detection," *IEEE Trans. Pattern Anal. Mach. Intell.*, vol. PAMI-8, no. 6, pp. 679–698, 1986.

[5] R. O. Duda and P. E. Hart, "Use of the Hough transformation to detect lines and curves in pictures," *Commun. ACM*, vol. 15, no. 1, pp. 11–15, 1972.

[6] M. Bojarski *et al.*, "End to end learning for self-driving cars," arXiv:1604.07316, 2016.

[7] F. Codevilla, M. Müller, A. López, V. Koltun, and A. Dosovitskiy, "End-to-end driving via conditional imitation learning," in *Proc. IEEE Int. Conf. Robotics and Automation (ICRA)*, 2018, pp. 4693–4700.

[8] B. Paden, M. Čáp, S. Z. Yong, D. Yershov, and E. Frazzoli, "A survey of motion planning and control techniques for self-driving urban vehicles," *IEEE Trans. Intell. Veh.*, vol. 1, no. 1, pp. 33–55, 2016.

[9] K. J. Åström and T. Hägglund, *PID Controllers: Theory, Design, and Tuning*, 2nd ed. Research Triangle Park, NC, USA: ISA, 1995.

[10] S. Thrun *et al.*, "Stanley: The robot that won the DARPA Grand Challenge," *J. Field Robotics*, vol. 23, no. 9, pp. 661–692, 2006.

[11] R. C. Coulter, "Implementation of the pure pursuit path tracking algorithm," Robotics Institute, Carnegie Mellon Univ., Pittsburgh, PA, USA, Tech. Rep. CMU-RI-TR-92-01, 1992.

*(Adil: add Lit-review sources here; renumber in order of first citation when merging.)*
