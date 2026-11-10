### Overall

| Scenario | Runs | Success rate | Collisions | Lane RMS err (m) | Lane max err (m) | Reaction dist (m) | Reaction time (s) | Min gap (m) |
|---|---|---|---|---|---|---|---|---|
| empty_lane | 4 | 3/4 (75%) | 1 | 1.02 (0.02-3.80) | 2.58 (0.04-9.53) | 0.23 | 0.00 | 0.23 |
| obstacle_stop | 4 | 0/4 (0%) | 4 | 0.05 (0.00-0.14) | 0.11 (0.01-0.31) | - | - | 26.00 (14.47-49.26) |
| obstacle_avoid | 4 | 1/4 (25%) | 3 | 0.02 (0.00-0.06) | 0.09 (0.00-0.29) | 29.27 | 2.85 | 34.34 (29.27-40.36) |

### Per town

| Town | empty_lane | obstacle_stop | obstacle_avoid | Collisions |
|---|---|---|---|---|
| Town03_Opt | 1/2 | 0/2 | 1/2 | 4 |
| Town04 | 2/2 | 0/2 | 0/2 | 4 |

### Obstacle detection vs ground truth

| Scenario | Distance MAE (m) | Missed ticks | False-alarm ticks | Adjacent-lane disagree ticks | Lane conf (mean) | Lane lost ticks |
|---|---|---|---|---|---|---|
| empty_lane | - | 0 | 2 | 90 | 0.83 (0.51-0.94) | 49 |
| obstacle_stop | 10.51 (4.21-23.34) | 4 | 0 | 77 | 0.92 (0.88-0.95) | 13 |
| obstacle_avoid | 12.72 (5.87-17.41) | 4 | 56 | 80 | 0.80 (0.50-0.94) | 25 |
