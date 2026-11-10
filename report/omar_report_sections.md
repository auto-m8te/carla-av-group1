# Report sections — Omar (draft)

> Section V separates original Run A, final Run B, camera Run C and the noise test.
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

All experiments used CARLA 0.9.16 [3] in synchronous mode with a 0.05 s tick (20 Hz). The evaluations are separate datasets, not repeated measurements of the same final code.

Run A is the original 90-run evaluation with ground-truth obstacle and adjacent-lane perception and map-waypoint steering: three core scenarios, six towns and five seeds per combination. Its original tables are retained below. Starts had at most 25° heading change, obstacle distances were 35–55 m and cruise speeds 6–10 m/s.

Run B (FINAL) contains 200 runs using LiDAR obstacle and adjacent-lane detection with map-waypoint steering. Eight scenarios were tested across Town01, Town02_Opt, Town03_Opt, Town04, Town05_Opt and Town10HD. Coverage is uneven: core scenarios have 30 runs each, curves 20 each, lead scenarios 25 each and oncoming overtakes 20. A dash in the town table means untested, not failure. Town02/03/05 use layered _Opt maps because standard-map renderer crashes prevented reliable execution on this machine.

Run C uses camera lane steering plus LiDAR on Town04, Town03_Opt and Town01: three core scenarios, two runs per combination (18 runs). A separate noise test adds 0.05 m LiDAR range noise on Town04 over 15 core-scenario runs. Neither is a repeat of the full Run B matrix. Sensor reads are synchronised to the simulation frame.

### B. Metrics

Success requires no collision. Empty-lane and curve-lane runs require CRUISE lateral offset below 1.0 m. Stop, curve-stop and lead-brake runs require final speed below 0.2 m/s and minimum gap ≥2 m. Lead-vehicle runs finish after 120 m and require minimum gap ≥2 m. Avoidance requires AVOID → PASSING → RETURN and final offset below 0.5 m. Oncoming success additionally requires never more than half a lane of displacement while the other car approaches (identified by a shrinking logged gap). A safe aborted attempt alone does not count as completed avoidance.

Lane RMS and maximum error use CRUISE ticks only; they do not measure tracking throughout FOLLOW or a lane change. Reaction distance is the gap at the first qualifying departure from CRUISE; reaction time is measured from first detection, not sensor processing latency. Minimum gap is the nearest reported in-path bumper gap. Table I retains mean (min–max); Table III reports scenario means, including the mean of each run's maximum lane error. Ground truth is logged for detection-error comparison.

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


Run B completed 194/200 runs with zero collisions. Six runs failed completion criteria; zero collisions does not mean every task completed.

**Table III(a) – Run B per scenario (means; lane maxima averaged across runs)**

| Scenario | Success | Coll. | RMS m | Max m | React. m | React. s | Gap m |
|---|---|---|---|---|---|---|---|
| empty_lane | 30/30 | 0 | 0.03 | 0.11 | - | - | - |
| obstacle_stop | 30/30 | 0 | 0.03 | 0.05 | 11.09 | 6.37 | 6.99 |
| obstacle_avoid | 30/30 | 0 | 0.04 | 0.30 | 26.65 | 3.68 | 13.56 |
| curve_lane | 20/20 | 0 | 0.17 | 0.27 | - | - | - |
| curve_stop | 20/20 | 0 | 0.14 | 0.24 | 11.52 | 6.12 | 7.44 |
| lead_vehicle | 25/25 | 0 | 0.00 | 0.01 | 28.58 | 1.40 | 9.62 |
| lead_brake | 21/25 | 0 | 0.00 | 0.01 | 28.87 | 1.40 | 6.01 |
| oncoming_overtake | 18/20 | 0 | 0.20 | 0.50 | 20.96 | 5.17 | 6.36 |

**Table III(b) – Run B per town**

| Town | EL | OS | OA | CL | CS | LV | LB | OO | Coll. |
|---|---|---|---|---|---|---|---|---|---|
| Town01 | 5/5 | 5/5 | 5/5 | - | - | 5/5 | 3/5 | 5/5 | 0 |
| Town02_Opt | 5/5 | 5/5 | 5/5 | - | - | 5/5 | 4/5 | 3/5 | 0 |
| Town03_Opt | 5/5 | 5/5 | 5/5 | 5/5 | 5/5 | 5/5 | 5/5 | 5/5 | 0 |
| Town04 | 5/5 | 5/5 | 5/5 | 5/5 | 5/5 | 5/5 | 5/5 | - | 0 |
| Town05_Opt | 5/5 | 5/5 | 5/5 | 5/5 | 5/5 | 5/5 | 4/5 | - | 0 |
| Town10HD | 5/5 | 5/5 | 5/5 | 5/5 | 5/5 | - | - | 5/5 | 0 |

EL = empty_lane; OS = obstacle_stop; OA = obstacle_avoid; CL = curve_lane; CS = curve_stop; LV = lead_vehicle; LB = lead_brake; OO = oncoming_overtake.

**Table IV – Run C: camera steering plus LiDAR, success per town**

| Town | empty_lane | obstacle_stop | obstacle_avoid | Collisions |
|---|---|---|---|---|
| Town01 | 2/2 | 2/2 | 2/2 | 0 |
| Town03_Opt | 1/2 | 2/2 | 2/2 | 1 |
| Town04 | 2/2 | 2/2 | 2/2 | 0 |

Run C succeeded in 17/18 runs, with one collision: Town03_Opt empty_lane run 1 drifted off the road. Both obstacle scenarios succeeded in 6/6 runs; empty_lane succeeded in 5/6.

**Table V – Offline camera lane detector, held-out drive 2**

| Town | N | Detection | Offset MAE m | Heading MAE ° | Offset sign | Steering sign |
|---|---|---|---|---|---|---|
| Town01 | 93 | 92.5% | 0.350 | 123.73 | 34.5% | 38.2% |
| Town02_Opt | 93 | 88.2% | 0.190 | 2.56 | 75.0% | 81.7% |
| Town03_Opt | 93 | 95.7% | 0.138 | 2.63 | 91.1% | 83.9% |
| Town04 | 93 | 90.3% | 0.251 | 4.36 | 81.2% | 97.1% |
| Town05_Opt | 93 | 90.3% | 0.147 | 0.84 | 81.8% | 80.3% |
| Town10HD | 93 | 90.3% | 0.517 | 9.50 | 36.5% | 52.4% |
| Overall | 558 | 91.2% | 0.265 | 24.20 | 67.2% | 73.2% |

The labelled dataset contains 1,674 frames: drives 0–1 (1,116 frames) were used for tuning and drive 2 (558 frames) was held out. Frames with |true offset| >1.5 m are excluded. MAE uses detected frames; sign metrics use |true offset| >0.2 m. Missed detections count as wrong offset signs; steering includes loss decay. Overall values pool frames. The test offset MAE of 0.265 m rounds to 0.27 m.

With 0.05 m LiDAR range noise, Town04 achieved 15/15 successes and zero collisions. Distance MAE was 1.14 m for obstacle_stop and 1.02 m for obstacle_avoid (about 1.1 m). This is a small sensitivity test, not a full noisy-sensor evaluation.

*Fig. 2 – figures/Town04_curve_stop_run1.png: Run B braking behind an obstacle on a curve.*

*Fig. 3 – figures/Town04_lead_vehicle_run1.png: Run B FOLLOW behind a moving lead vehicle.*

*Fig. 4 – figures/Town01_oncoming_overtake_run1.png: Run B waits for oncoming traffic and then overtakes.*

### D. Discussion

The core stop and avoid scenarios, both curve scenarios, and lead_vehicle all completed without collisions. These results support the tested controller with LiDAR and map steering. They do not establish camera robustness across the full scenario set.

Lead_brake succeeded in 21/25 runs. The failed runs remained about 5–6 m behind the lead but did not reach the final stopped-speed threshold within the time limit. Across this scenario, the mean minimum gap was 6.01 m (5.24–6.79 m). The controller avoided contact, but its low-speed convergence needs improvement.

Oncoming_overtake succeeded in 18/20 runs. Both failures were in Town02_Opt. Run 2 has waited_for_oncoming=True and did not complete the manoeuvre. Run 4 has waited_for_oncoming=False in per_run.csv, which conflicts with the description that both runs remained safe. The stored flag uses the half-lane approach criterion, so the supplied data do not support claiming that every oncoming run was safe. This discrepancy needs review before a stronger safety claim.

Curve_stop had LiDAR distance MAE 2.63 m, larger than the straight stop value of 0.47 m. The comparison mixes lane-arc ground-truth distance with straight-line LiDAR distance on curves; part of the error is geometric rather than a missed obstacle. All 20 curve-stop runs passed, with minimum gaps of 2.57–10.21 m.

Camera closed-loop success was 17/18, with the Town03_Opt road departure. Offline detection was 91.2%, but steering-sign agreement was only 73.2%. Town01 and Town10HD were weak: offset MAE 0.350 and 0.517 m, and steering-sign agreement 38.2% and 52.4%. Town01 heading MAE remained 123.73°; junction cross-road edges also survived fallback in Town10HD. Detection alone is therefore insufficient evidence of correct steering.

**What did not work at first (development iterations).** The original twelve rows are retained; subsequent problems and fixes are drawn from git history after 9c0a7ed, with safety and timing problems first.

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
| 13 | Delayed braking and collisions with sensors under load | Sensor callbacks returned old simulation frames | Bounded wait for the current frame; reuse the same LiDAR sweep within a tick |
| 14 | Moving lead was overtaken and cut off during merge-back | Motion confirmation delayed the no-overtake decision | Block AVOID as soon as the speed estimate suggests motion, before confirmation |
| 15 | Oncoming cars at 60–80 m disappeared from LiDAR | Empty range bins reset road height and cars were filtered as road | Retain the last road height through empty bins; extend moving-traffic LiDAR to 185 m |
| 16 | Pull-out became unsafe when adjacent traffic approached | A distance-only adjacent-lane check ignored closing speed | Project clearance over 9 s; require 1 s continuously free; ABORT home if clearance closes |
| 17 | Parked cars briefly triggered FOLLOW after spawning | Spawn settling looked like obstacle motion | Confirm motion over 0.5 s and after 3 m driven |
| 18 | Home lane could be the neighbouring lane at spawn | Vehicle snapped sideways while dropping from spawn height | Anchor home lane to the scenario start waypoint |
| 19 | Ego could not pull out after waiting for oncoming traffic | Stopping too close left insufficient steering room | Wait 10 m back; use stronger low-speed pull-out gains and creep recovery |
| 20 | Camera selected road edges or inconsistent boundary pairs | Colour-only masks and weak geometry checks accepted wrong edges | Local contrast, yellow and curb masks; wider ROI; pair checks and lane-width prior |
| 21 | Camera steering oscillated between estimates | Individual boundary estimates were unstable | Median filter and steering rate limit |

**Limitations.**

Run A evaluates the original static-obstacle code. Run B adds moving leads and oncoming traffic, but does not cover pedestrians, dense mixed traffic or arbitrary interactions. Its steering and LiDAR corridor still depend on map geometry. Run C demonstrates camera integration on a small core-scenario subset; offline town-dependent errors and the closed-loop collision remain unresolved. The noise experiment covers only Town04 and one range-noise setting, without real sensor artefacts or adverse weather. Five seeded runs per available town/scenario provide limited coverage, and skipped road configurations are not successes. _Opt maps avoid renderer failures but limit claims about standard-map execution. Junction route planning and navigation are not implemented; this is Khalid's E5 card. Gentle lane changes and conservative clearance checks can delay or prevent completion.

---

## VI. Conclusion & Future Work  *(shared: Adil + Abir + Omar)*

This project implemented lane following, LiDAR obstacle detection and a stop/follow/avoid controller in CARLA. The original ground-truth Run A achieved 90/90 successes with zero collisions. Final Run B achieved 194/200 successes with zero collisions across eight scenarios and six towns using LiDAR and map-waypoint steering. Camera steering plus LiDAR achieved 17/18 successes with one collision. The offline held-out lane-detector test achieved 91.2% detection, 0.27 m offset MAE and 73.2% steering-sign agreement. The Town04 LiDAR noise test achieved 15/15 successes. These results show that the tested LiDAR and controller combination handled the selected cases, while camera robustness and timely task completion remain limitations.

Future work:

Improve low-speed stopping convergence and safe oncoming-manoeuvre completion; repair camera boundary and heading errors across towns; expand camera and noisy-sensor tests to moving traffic, weather and lighting changes; implement junction navigation (Khalid's E5 card); and compare smoother planned lane-change trajectories [8] with the current PD-style controller.

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
