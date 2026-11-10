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
