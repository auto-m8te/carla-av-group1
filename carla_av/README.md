# CARLA lane-following + obstacle avoidance — Group 1

Khalid · Abir · Adil · Prithiv · Omar — Engineering Teamwork, WS2026/27 SRH Berlin

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
disagreements. Lane steering is still the ground-truth fallback until Adil's module lands.

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

- `lead_vehicle`: a car 30–40 m ahead drives at 40–60% of the ego target
  speed. Ego follows until it has driven 120 m. Success requires no collision
  and a minimum bumper gap of 2 m.
- `lead_brake`: the same lead stops after driving 40 m; a parked adjacent-lane
  blocker starts 50–60 m ahead of ego. The run ends after ego holds stopped
  for 3 s. Success requires no collision, final speed below 0.2 m/s and a
  minimum gap of 2 m.
- `oncoming_overtake`: a parked obstacle 35–55 m ahead and an oncoming car
  another 40–60 m beyond it, travelling at 6–8 m/s. Only opposite-direction
  adjacent lanes (different lane-id signs) qualify; unavailable roads are
  warned about and skipped. Ego waits, then completes AVOID → PASSING → RETURN.
  Success requires no collision, those three modes, a final offset below 0.5 m,
  and the first AVOID tick after the oncoming car has passed. Analysis records
  `waited_for_oncoming`, approximating passage by an increasing `other_gap`.

`GapTracker` estimates signed obstacle speed from ego speed plus the gap
derivative, smoothed with alpha 0.3. It needs three samples and resets on
missing detections or jumps over 3 m. Ground-truth tracking interpolates the
waypoint gap to avoid differentiating 1 m quantisation; reported distances keep
their original definition. Negative obstacle speed means oncoming traffic.

For a known lead speed above 1 m/s, FOLLOW uses a desired gap of
`6 + 1.8 × ego_speed` metres and a target speed of
`clamp(lead_speed + 0.4 × (gap − desired_gap), 0, cruise_speed)`.
It brakes for a speed excess above 1 m/s or a closing TTC below 3 s, and never
starts an overtake of that moving lead. At 1 m/s or below, the original
stop/avoid rules apply; emergency braking retains priority.
Moving runs use the 6 m standstill gap as their braking margin, keeping a
waiting ego outside the 5 m emergency zone so it can resume after traffic passes.
The lead-brake blocker is placed 20 m beyond the lead's initial position
(50–60 m from ego), keeping it beside the waiting ego when the lead stops.

Adjacent traffic beside ego (15 m behind through 10 m beyond its front bumper)
blocks a lane change immediately. Farther ahead, the projected gap after
`2 × lane_change_time + 4` seconds (9 s by default) must exceed 10 m.
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
