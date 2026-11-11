### Overall

| Scenario | Runs | Success rate | Collisions | Lane RMS err (m) | Lane max err (m) | Reaction dist (m) | Reaction time (s) | Min gap (m) |
|---|---|---|---|---|---|---|---|---|
| route | 30 | 12/30 (40%) | 1 | 0.30 (0.09-0.72) | 1.67 (0.25-5.19) | 6.58 (0.14-11.96) | 1.51 (0.00-13.70) | 7.21 (0.00-14.02) |
| route_obstacle | 30 | 29/30 (97%) | 1 | 0.54 (0.00-0.90) | 1.71 (0.00-2.29) | 11.66 (8.02-23.37) | 8.45 (0.00-29.20) | 8.36 (2.43-10.41) |

### Per town

| Town | route | route_obstacle | Collisions |
|---|---|---|---|
| Town01 | 4/5 | 5/5 | 0 |
| Town02_Opt | 0/5 | 4/5 | 2 |
| Town03_Opt | 3/5 | 5/5 | 0 |
| Town04 | 3/5 | 5/5 | 0 |
| Town05_Opt | 0/5 | 5/5 | 0 |
| Town10HD | 2/5 | 5/5 | 0 |

### Obstacle detection vs ground truth

| Scenario | Distance MAE (m) | Missed ticks | False-alarm ticks | Adjacent-lane disagree ticks | Lane conf (mean) | Lane lost ticks |
|---|---|---|---|---|---|---|
| route | - | 0 | 2040 | 4661 | - | 0 |
| route_obstacle | 3.01 (0.45-7.71) | 711 | 594 | 4348 | - | 0 |

### Navigation

| Scenario | Route length (m) | Junctions passed | Time (s) | Mean speed (m/s) |
|---|---|---|---|---|
| route | 348.85 (256.99-437.88) | 3.67 (0.00-6.00) | 55.05 (5.55-90.00) | 5.85 (0.31-7.38) |
| route_obstacle | 342.97 (262.48-448.31) | 2.70 (0.00-8.00) | 38.38 (2.80-90.00) | 4.62 (1.93-5.97) |
