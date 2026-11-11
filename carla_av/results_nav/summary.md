### Overall

| Scenario | Runs | Success rate | Collisions | Lane RMS err (m) | Lane max err (m) | Reaction dist (m) | Reaction time (s) | Min gap (m) |
|---|---|---|---|---|---|---|---|---|
| route | 5 | 2/5 (40%) | 0 | 0.30 (0.25-0.38) | 1.41 (0.68-1.92) | - | - | - |
| route_obstacle | 5 | 5/5 (100%) | 0 | 0.53 (0.00-0.78) | 1.53 (0.02-2.14) | 11.70 (10.46-12.57) | 8.23 (6.30-9.65) | 8.39 (5.83-10.11) |

### Per town

| Town | route | route_obstacle | Collisions |
|---|---|---|---|
| Town10HD | 2/5 | 5/5 | 0 |

### Obstacle detection vs ground truth

| Scenario | Distance MAE (m) | Missed ticks | False-alarm ticks | Adjacent-lane disagree ticks | Lane conf (mean) | Lane lost ticks |
|---|---|---|---|---|---|---|
| route | - | 0 | 0 | 379 | - | 0 |
| route_obstacle | 2.90 (0.45-5.90) | 102 | 13 | 847 | - | 0 |

### Navigation

| Scenario | Route length (m) | Junctions passed | Time (s) | Mean speed (m/s) |
|---|---|---|---|---|
| route | 353.18 (300.07-418.53) | 4.00 (3.00-5.00) | 56.26 (44.55-64.65) | 6.10 (5.65-6.59) |
| route_obstacle | 342.38 (271.49-431.88) | 2.40 (1.00-4.00) | 42.18 (15.75-90.00) | 4.26 (3.63-4.75) |
