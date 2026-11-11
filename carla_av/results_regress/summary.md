### Overall

| Scenario | Runs | Success rate | Collisions | Lane RMS err (m) | Lane max err (m) | Reaction dist (m) | Reaction time (s) | Min gap (m) |
|---|---|---|---|---|---|---|---|---|
| empty_lane | 10 | 10/10 (100%) | 0 | 0.01 (0.00-0.10) | 0.03 (0.00-0.22) | - | - | - |
| obstacle_stop | 10 | 10/10 (100%) | 0 | 0.01 (0.00-0.05) | 0.02 (0.00-0.09) | 10.70 (9.96-11.98) | 6.17 (4.95-8.60) | 7.30 (5.83-10.32) |
| obstacle_avoid | 10 | 10/10 (100%) | 0 | 0.04 (0.03-0.05) | 0.30 (0.29-0.30) | 25.76 (24.12-30.67) | 3.27 (2.05-6.10) | 13.14 (10.39-16.49) |

### Per town

| Town | empty_lane | obstacle_stop | obstacle_avoid | Collisions |
|---|---|---|---|---|
| Town01 | 5/5 | 5/5 | 5/5 | 0 |
| Town04 | 5/5 | 5/5 | 5/5 | 0 |

### Obstacle detection vs ground truth

| Scenario | Distance MAE (m) | Missed ticks | False-alarm ticks | Adjacent-lane disagree ticks | Lane conf (mean) | Lane lost ticks |
|---|---|---|---|---|---|---|
| empty_lane | - | 0 | 0 | 190 | - | 0 |
| obstacle_stop | 0.74 (0.19-2.36) | 0 | 0 | 169 | - | 0 |
| obstacle_avoid | 0.68 (0.31-1.97) | 33 | 0 | 190 | - | 0 |
