### Overall

| Scenario | Runs | Success rate | Collisions | Lane RMS err (m) | Lane max err (m) | Reaction dist (m) | Reaction time (s) | Min gap (m) |
|---|---|---|---|---|---|---|---|---|
| empty_lane | 6 | 5/6 (83%) | 1 | 0.62 (0.01-3.44) | 1.61 (0.04-8.72) | 0.93 | 0.00 | 0.12 |
| obstacle_stop | 6 | 6/6 (100%) | 0 | 0.06 (0.01-0.11) | 0.12 (0.01-0.20) | 11.31 (10.29-12.21) | 6.37 (5.30-8.55) | 7.38 (6.07-9.80) |
| obstacle_avoid | 6 | 6/6 (100%) | 0 | 0.08 (0.03-0.20) | 0.30 (0.29-0.33) | 25.62 (24.12-27.96) | 3.74 (2.40-6.10) | 12.70 (10.61-13.73) |

### Per town

| Town | empty_lane | obstacle_stop | obstacle_avoid | Collisions |
|---|---|---|---|---|
| Town01 | 2/2 | 2/2 | 2/2 | 0 |
| Town03_Opt | 1/2 | 2/2 | 2/2 | 1 |
| Town04 | 2/2 | 2/2 | 2/2 | 0 |

### Obstacle detection vs ground truth

| Scenario | Distance MAE (m) | Missed ticks | False-alarm ticks | Adjacent-lane disagree ticks | Lane conf (mean) | Lane lost ticks |
|---|---|---|---|---|---|---|
| empty_lane | - | 0 | 4 | 123 | 0.85 (0.52-0.96) | 41 |
| obstacle_stop | 0.84 (0.16-1.74) | 0 | 0 | 108 | 0.93 (0.80-0.99) | 0 |
| obstacle_avoid | 0.66 (0.32-2.01) | 21 | 0 | 123 | 0.78 (0.53-0.87) | 130 |
