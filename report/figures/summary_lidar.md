### Overall

| Scenario | Runs | Success rate | Collisions | Lane RMS err (m) | Lane max err (m) | Reaction dist (m) | Reaction time (s) | Min gap (m) |
|---|---|---|---|---|---|---|---|---|
| empty_lane | 30 | 30/30 (100%) | 0 | 0.02 (0.00-0.12) | 0.10 (0.00-0.80) | - | - | 33.63 |
| obstacle_stop | 30 | 30/30 (100%) | 0 | 0.02 (0.00-0.20) | 0.04 (0.00-0.34) | 11.05 (9.51-12.53) | 6.36 (4.30-9.65) | 7.02 (5.41-10.32) |
| obstacle_avoid | 30 | 30/30 (100%) | 0 | 0.04 (0.03-0.20) | 0.30 (0.29-0.30) | 26.85 (23.95-33.18) | 3.50 (0.45-6.10) | 13.78 (10.19-19.19) |

### Per town

| Town | empty_lane | obstacle_stop | obstacle_avoid | Collisions |
|---|---|---|---|---|
| Town01 | 5/5 | 5/5 | 5/5 | 0 |
| Town02_Opt | 5/5 | 5/5 | 5/5 | 0 |
| Town03_Opt | 5/5 | 5/5 | 5/5 | 0 |
| Town04 | 5/5 | 5/5 | 5/5 | 0 |
| Town05 | 5/5 | 5/5 | 5/5 | 0 |
| Town10HD | 5/5 | 5/5 | 5/5 | 0 |

### Obstacle detection vs ground truth

| Scenario | Distance MAE (m) | Missed ticks | False-alarm ticks | Adjacent-lane disagree ticks |
|---|---|---|---|---|
| empty_lane | - | 0 | 65 | 97 |
| obstacle_stop | 0.48 (0.21-1.97) | 131 | 0 | 23 |
| obstacle_avoid | 0.44 (0.27-1.76) | 242 | 71 | 263 |
