### Overall

| Scenario | Runs | Success rate | Collisions | Lane RMS err (m) | Lane max err (m) | Reaction dist (m) | Reaction time (s) | Min gap (m) |
|---|---|---|---|---|---|---|---|---|
| empty_lane | 5 | 5/5 (100%) | 0 | 0.02 (0.00-0.10) | 0.05 (0.00-0.22) | - | - | - |
| obstacle_stop | 5 | 5/5 (100%) | 0 | 0.03 (0.00-0.05) | 0.05 (0.00-0.09) | 10.75 (10.16-11.04) | 6.18 (4.90-8.55) | 6.22 (6.08-6.46) |
| obstacle_avoid | 5 | 5/5 (100%) | 0 | 0.04 (0.03-0.05) | 0.30 (0.29-0.30) | 26.46 (24.14-30.56) | 3.09 (2.05-4.55) | 12.67 (10.35-16.08) |

### Per town

| Town | empty_lane | obstacle_stop | obstacle_avoid | Collisions |
|---|---|---|---|---|
| Town04 | 5/5 | 5/5 | 5/5 | 0 |

### Obstacle detection vs ground truth

| Scenario | Distance MAE (m) | Missed ticks | False-alarm ticks | Adjacent-lane disagree ticks | Lane conf (mean) | Lane lost ticks |
|---|---|---|---|---|---|---|
| empty_lane | - | 0 | 0 | 95 | - | 0 |
| obstacle_stop | 1.14 (0.24-2.35) | 0 | 0 | 83 | - | 0 |
| obstacle_avoid | 1.02 (0.33-1.99) | 18 | 0 | 95 | - | 0 |
