# CARLA lane-following + obstacle avoidance â€” Group 1

Khalid Â· Abir Â· Adil Â· Prithiv Â· Omar â€” Engineering Teamwork, WS2026/27 SRH Berlin

## Layout
```
avoidance/collision_avoidance.py  stop vs. steer-around state machine (pure Python, no CARLA)  [Omar]
avoidance/carla_adapter.py        CARLA <-> module glue; plug-in points for lane + obstacle modules
perception/obstacle_detection.py  LiDAR point cloud -> obstacle_distance + "ahead" flag (numpy, no CARLA)  [E3 card: Abir]
perception/lidar_sensor.py        roof LiDAR on the ego car, feeds the detector each tick             [E3 card: Abir]
scenarios/run_scenarios.py        runs empty_lane / obstacle_stop / obstacle_avoid, logs CSVs   [Omar + Abir]
analysis/analyze_results.py       metrics table + plots for the Findings section              [Omar]
tests/                            pytest: unit + closed-loop toy sim + analysis + LiDAR detector
```

## Interfaces
| From | Value | Type |
|---|---|---|
| Lane module (Adil) | `steering_angle` | float in [-1, 1] |
| Obstacle module (Abir) | `obstacle_distance` | float metres to nearest obstacle in path, `None` if clear |
| Avoidance (Omar) | `ControlCommand(throttle, steer, brake, mode)` | applied by integration loop (Prithiv) |

Plug real modules in via `CarlaAvoidanceBridge(world, ego, cfg, lane_steer_fn=..., obstacle_distance_fn=...)`.
Without them, ground-truth fallbacks are used (say so in the report).

The LiDAR module is already wired in:
- `--obstacle lidar`: `obstacle_distance` from LiDAR
- `--adjacent lidar`: the "is the lane beside us free?" check from LiDAR too

Each run also logs the ground-truth values (`gt_obstacle_distance`, `gt_adjacent_lane_clear`),
and the analysis reports distance error, missed ticks, false alarms and adjacent-lane
disagreements. Camera lane steering is available with `--lane camera`; ground truth remains the default.

## Obstacle detection (LiDAR)
```
32-channel roof LiDAR, 1 sweep per tick
  -> range gate (front bumper .. 60 m)
  -> road removal: road height per 2 m bin from points near the path (grade-limited)
  -> path corridor: lane ahead from the map, car width + 0.3 m each side
  -> cluster check: >= 3 points within 1 m depth (rejects noise)
  -> obstacle_distance = nearest cluster - front bumper   (None = clear)

Adjacent lane: same filter over the whole neighbouring lane, 15 m behind .. 60 m ahead,
own car body excluded. No sweep yet -> "not clear" (never swerve blind).
```

## Decision logic
```
          obstacle in path?
            | no -> CRUISE (follow lane)
            | yes
   dist <= emergency (5 m)? --yes--> EMERGENCY_STOP (full brake)
            | no
   dist <= stopping_dist + lookahead AND adjacent lane clear?
            | yes -> AVOID -> PASSING -> RETURN -> CRUISE
            | no
   dist <= stopping_dist? --yes--> BRAKE -> STOPPED (holds until lane/path clears)
            | no -> CRUISE (eased throttle)

stopping_dist = v * t_react + v^2 / (2 * a_max) + margin
```

## Run
```bash
uv venv --python 3.11 .venv && uv pip install -r requirements.txt   # carla==0.9.16 needs Python 3.10-3.12
python -m pytest -q                       # no CARLA needed
# start CarlaUE4 / CarlaUnreal server, then:
python -m scenarios.run_scenarios --runs 5            # all 6 towns, 90 runs (~30 min)
python -m scenarios.run_scenarios --towns Town03 --runs 1   # quick single-town check
python -m scenarios.run_scenarios --runs 5 --obstacle lidar --adjacent lidar --out results_lidar   # LiDAR perception
python -m scenarios.run_scenarios --towns Town04 --runs 1 --realtime --hud --out results_demo   # live demo
python -m analysis.analyze_results --results results
```


## Sharp-curve scenarios (E5)

`curve_lane` follows a sharp curve without obstacles and finishes after 100 m.
`curve_stop` places a parked car at the half-heading apex (25-55 m from the
start), with a second parked car blocking the adjacent lane, and must stop.
Curves accumulate 45-120 degrees of heading change within the first 80 m;
consecutive 5 m probes must turn at most 25 degrees. The stretch must have no
junctions and an adjacent driving lane on the avoidance side. Towns without
an available usable curve are warned about and skipped.

```bash
python -m scenarios.run_scenarios --scenario curves --towns Town03 Town04 --runs 5
```

`--scenario all` retains the original three scenarios. `--scenario curves`
runs both curve scenarios; `--scenario everything` runs all eight. Each curve
scenario can also be selected by name. Analysis applies the lane-following
success rule to `curve_lane` and the stopping success rule to `curve_stop`.

## Moving traffic (E3 / E4 / E5)

```bash
python -m scenarios.run_scenarios --scenario moving --towns Town01 Town02 --runs 5
```

`--scenario moving` selects these three seeded scenarios (each is also available
by name). `--scenario all` continues to select the original three scenarios.

- `lead_vehicle`: a car 30â€“40 m ahead drives at 40â€“60% of the ego target
  speed. Ego follows until it has driven 120 m. Success requires no collision
  and a minimum bumper gap of 2 m.
- `lead_brake`: after driving 40 m, the same lead ramps its target speed down
  at 3 m/sÂ², tracked by the P speed controller; full brake holds the stop only
  below 0.3 m/s. A parked adjacent-lane blocker sits beside the planned stop:
  lead start + 40 m + 8 m braking allowance, clamped to the usable road length.
  The road must extend at least 20 m beyond this planned position.
  The run ends after ego holds stopped
  for 3 s. Success requires no collision, final speed below 0.2 m/s and a
  minimum gap of 2 m.
- `oncoming_overtake`: a parked obstacle 35â€“55 m ahead and an oncoming car
  another 40â€“60 m beyond it, travelling at 6â€“8 m/s. Only opposite-direction
  adjacent lanes (different lane-id signs) qualify; unavailable roads are
  warned about and skipped. Ego waits, then completes AVOID â†’ PASSING â†’ RETURN.
  Success requires no collision, those three modes, a final offset below 0.5 m,
  and the first AVOID tick after the oncoming car has passed. Analysis records
  `waited_for_oncoming`, approximating passage by an increasing `other_gap`.

`GapTracker` estimates signed obstacle speed from ego speed plus the gap
derivative, smoothed with alpha 0.3. It needs three samples and resets on
missing detections or jumps over 3 m. Ground-truth tracking interpolates the
waypoint gap to avoid differentiating 1 m quantisation; reported distances keep
their original definition. Negative obstacle speed means oncoming traffic.

For a known lead speed above 1 m/s, FOLLOW uses a desired gap of
`6 + 1.8 Ã— ego_speed` metres and a target speed of
`clamp(lead_speed + 0.4 Ã— (gap âˆ’ desired_gap), 0, cruise_speed)`.
It brakes for a speed excess above 1 m/s or a closing TTC below 3 s, and never
starts an overtake of that moving lead. At 1 m/s or below, the original
stop/avoid rules apply; emergency braking retains priority.
Moving runs use the 6 m standstill gap as their braking margin, keeping a
waiting ego outside the 5 m emergency zone so it can resume after traffic passes.
The lead-brake blocker is placed 20 m beyond the lead's initial position
(50â€“60 m from ego), keeping it beside the waiting ego when the lead stops.

Adjacent traffic beside ego (15 m behind through 10 m beyond its front bumper)
blocks a lane change immediately. Farther ahead, the projected gap after
`2 Ã— lane_change_time + 4` seconds (9 s by default) must exceed 10 m.
Unknown speed for a visible object blocks the manoeuvre. LiDAR estimates speed
with a separate tracker; ground truth projects actor velocity along the home
lane. Moving runs extend adjacent sensing to 180 m (LiDAR range 185 m) so an
oncoming car can be seen before pull-out; existing scenarios retain 60 m.

CSV files append `obstacle_speed`, `other_speed`, and `other_gap` after the
existing columns. Speeds are m/s; gaps are bumper gaps in metres. The scripted
actor's gap follows the road for lead scenarios and uses straight-line distance
for oncoming traffic; absent values are blank. Moving actors use waypoint
steering and a small speed controller and share the existing safe teardown.
All runs retain the 50 s timeout. Tests use toy models and actor doubles without
starting or connecting to CARLA; driving behaviour still needs simulator runs.

- Empty LiDAR range bins hold the previous road height, preserving distant car detection when road returns are sparse.
- ABORT steers committed lane changes home while braking when the target lane becomes blocked and at least 8 m of bumper clearance remains.

- Adjacent clearance from either source requires `adjacent_clear_hold=0.5` s continuously free; blocked answers are immediate, CSV clearance is debounced and ground truth stays raw.
- AVOID caps the speed target while mostly in the home lane at `max(avoid_min_speed=2.0, (gap - safety_margin) / lane_change_time)` and brakes at 0.3 above the target by more than 1 m/s.

## Lane detection (camera)

`--lane camera` uses an 800x450 BGR camera (90 degree horizontal FOV,
1.5 m forward, 1.6 m high, pitched down 8 degrees). The classical pipeline is:

1. Detect white markings by local value contrast against a 41-pixel Gaussian
   background, retaining the yellow HSV mask. Add strong grayscale Canny edges
   for unpainted curbs.
2. Apply a wide trapezoidal ROI (full width at the bottom), find Hough segments,
   and reject horizontal, distant, implausible-bottom and implausible-heading
   candidates. Cluster duplicate edges with length-weighted fits.
3. Require negative/positive image slopes for left/right boundaries, bottom
   intersections bracketing the image centre, width within 40% of the prior,
   and a vanishing point above the ROI. Prefer pairs matching the pixel-width
   prior, learned as the median of the last 31 accepted pairs per sequence.
   Use a heading/offset-validated single boundary with the 3.5 m width prior
   when no pair survives. Two boundaries give confidence 1.0; one gives 0.5.
4. Project onto a flat road using the camera geometry, extrapolate the centre to
   the vehicle origin, and estimate metric lateral offset and heading. Positive
   offset/heading means right. Smooth these estimates with `LaneSteering` and
   apply signed PD steering clamped to [-1, 1]. After five valid estimates,
   use their rolling median before exponential smoothing and limit command
   changes to 0.15 per update. Keep the existing startup response; after the
   first update, cap heading authority to half the lateral term for offsets
   exceeding 0.1 m. Lost detections give confidence
   zero and decay steering by 0.9; missing camera frames hold the command.

Thresholds and selection decisions used only drives 0 and 1 (tune). Drive 2 was
held out until the detector was frozen. Offline evaluation (no CARLA required):

```bash
.venv/Scripts/python.exe tools/eval_lane_dataset.py --save-overlays 3
```

This writes per-town and overall results to `../data/lane/eval.md` and three
annotated PNGs per town and split to `../data/lane/overlays/`. `--split tune`
allows tuning without reading test images; `--dataset PATH` selects the dataset.
Controllers reset for each town/drive and process frames in tick order.
Frames with absolute true offset above 1.5 m are ignored. MAEs use detected
frames; both sign metrics use absolute true offset above 0.2 m. Missed detections
count as incorrect offset signs; steering includes the controller's loss decay.

| Split | Frames | Detection | Offset MAE (m) | Heading MAE (deg) | Offset sign | Steering sign |
|---|---:|---:|---:|---:|---:|---:|
| Tune | 1116 | 85.0% | 0.240 | 6.60 | 71.9% | 82.2% |
| Test | 558 | 91.2% | 0.265 | 24.20 | 67.2% | 73.2% |

The overall test detection target (85%) and offset MAE target (0.30 m) are met.
Offset-sign (85%) and steering-sign (90%) targets are **not met**. Per-town test
failures: Town01 and Town10HD miss offset MAE and both sign targets;
Town02_Opt and Town05_Opt miss both sign targets; Town03_Opt misses steering
sign; Town04 misses offset sign. All towns meet the detection target.
Town01 heading MAE remains 123.73 degrees; geometric rejection has not resolved
junction ambiguity or reference headings far from the visible straight road.
Town10HD still selects cross-road/adjacent boundaries in some junction frames.
Compared with the previous test run, offset MAE improves from 0.338 to 0.265 m
and steering sign from 69.1% to 73.2%, while detection falls from 95.2% to 91.2%
and offset sign falls from 67.5% to 67.2%. No further tuning used drive 2.
The confidence values indicate available boundary
support, not calibrated correctness probabilities. These results do not establish
closed-loop lane-keeping performance. Three lossless tune-frame fixtures from
different towns are included under `tests/data/lane/` for offset regression tests.

```bash
python -m scenarios.run_scenarios --lane camera --lane-debug lane_overlays --out results_camera
```

`--lane {groundtruth,camera}` defaults to `groundtruth`. Optional `--lane-debug DIR`
saves an overlay every 20 calls in town/run subdirectories. CSVs append
`lane_conf`, `lane_offset_cam` (blank when unavailable), and `gt_lane_steer`
(the waypoint steering each tick). Ground-truth runs leave camera fields blank.
Analysis includes mean camera confidence and lost-lane ticks in `per_run.csv`
and the perception table when camera confidence is available.

## Recording a demo video

With a CARLA 0.9.16 server already running, record chase-camera MP4s directly
from simulation ticks (no screen capture or GUI required):

```powershell
.venv/Scripts/python.exe -m scenarios.run_scenarios --towns Town03 --scenario all --runs 1 --record-video videos
```

Each run produces `videos/<town>_<scenario>_run<N>.mp4` at 1280�720 and
20 fps, including a two-second scenario title card and a HUD showing the
controller mode, speed, obstacle gap, and perception sources. `--record-video`
works independently of `--hud`; `--realtime` is optional because video timing
uses simulation time. Existing CSV output and `--resume` behavior still apply.

Join selected clips in the given order with one-second black separators:

```powershell
.venv/Scripts/python.exe -m tools.make_demo_reel --inputs videos/Town03_empty_lane_run1.mp4 videos/Town03_obstacle_stop_run1.mp4 videos/Town03_obstacle_avoid_run1.mp4 --out videos/demo_reel.mp4
```

All reel inputs must have the same dimensions and frame rate.
