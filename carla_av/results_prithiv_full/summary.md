### Overall

| Scenario | Runs | Success rate | Collisions | Lane RMS err (m) | Lane max err (m) | Reaction dist (m) | Reaction time (s) | Min gap (m) |
|---|---|---|---|---|---|---|---|---|
| empty_lane | 1 | 1/1 (100%) | 0 | 0.07 | 0.25 | - | - | - |
| obstacle_stop | 1 | 1/1 (100%) | 0 | 0.00 | 0.00 | 11.97 | 7.30 | 9.80 |
| obstacle_avoid | 1 | 1/1 (100%) | 0 | 0.04 | 0.29 | 26.25 | 2.55 | 11.80 |
| pedestrian_crossing | 1 | 1/1 (100%) | 0 | 0.00 | 0.00 | 14.18 | 0.00 | 9.96 |
| emergency_vehicle | 1 | 1/1 (100%) | 0 | 0.25 | 0.37 | - | -22.05 | 20.86 |

### Per town

| Town | empty_lane | obstacle_stop | obstacle_avoid | pedestrian_crossing | emergency_vehicle | Collisions |
|---|---|---|---|---|---|---|
| Town03_Opt | 1/1 | 1/1 | 1/1 | 1/1 | 1/1 | 0 |

### Obstacle detection vs ground truth

| Scenario | Distance MAE (m) | Missed ticks | False-alarm ticks | Adjacent-lane disagree ticks | Lane conf (mean) | Lane lost ticks |
|---|---|---|---|---|---|---|
| empty_lane | - | 0 | 0 | 65 | - | 0 |
| obstacle_stop | 0.23 | 0 | 0 | 21 | - | 0 |
| obstacle_avoid | 0.35 | 2 | 0 | 19 | - | 0 |
| pedestrian_crossing | 0.44 | 0 | 3 | 164 | - | 0 |
| emergency_vehicle | 0.97 | 63 | 3 | 123 | - | 0 |

### Traffic-rule check

| Scenario | Runs checked | Speeding (s) | Off-road (s) | Near misses | Red lights crossed |
|---|---|---|---|---|---|
| empty_lane | 1 | 0.0 | 0.0 | 0 | - |
| obstacle_stop | 1 | 0.0 | 0.0 | 0 | - |
| obstacle_avoid | 1 | 0.0 | 0.0 | 0 | - |
| pedestrian_crossing | 1 | 0.0 | 0.0 | 0 | - |
| emergency_vehicle | 1 | 0.0 | 0.0 | 0 | - |
