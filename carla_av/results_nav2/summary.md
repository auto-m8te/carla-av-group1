### Overall

| Scenario | Runs | Success rate | Collisions | Lane RMS err (m) | Lane max err (m) | Reaction dist (m) | Reaction time (s) | Min gap (m) |
|---|---|---|---|---|---|---|---|---|
| route | 30 | 18/30 (60%) | 0 | 0.27 (0.04-1.71) | 1.42 (0.33-5.16) | 7.98 (3.84-11.88) | 0.01 (0.00-0.10) | 7.23 (3.84-11.88) |
| route_obstacle | 30 | 30/30 (100%) | 0 | 0.33 (0.00-0.54) | 1.28 (0.00-1.91) | 11.01 (7.23-12.76) | 6.20 (0.00-10.70) | 8.00 (2.30-10.33) |

### Per town

| Town | route | route_obstacle | Collisions |
|---|---|---|---|
| Town01 | 5/5 | 5/5 | 0 |
| Town02_Opt | 5/5 | 5/5 | 0 |
| Town03_Opt | 2/5 | 5/5 | 0 |
| Town04 | 3/5 | 5/5 | 0 |
| Town05_Opt | 1/5 | 5/5 | 0 |
| Town10HD | 2/5 | 5/5 | 0 |

### Obstacle detection vs ground truth

| Scenario | Distance MAE (m) | Missed ticks | False-alarm ticks | Adjacent-lane disagree ticks | Lane conf (mean) | Lane lost ticks |
|---|---|---|---|---|---|---|
| route | - | 0 | 673 | 3522 | - | 0 |
| route_obstacle | 2.80 (0.27-7.06) | 982 | 75 | 2469 | - | 0 |

### Navigation

| Scenario | Route length (m) | Junctions passed | Time (s) | Mean speed (m/s) |
|---|---|---|---|---|
| route | 348.85 (256.99-437.88) | 3.87 (2.00-6.00) | 56.87 (40.15-90.00) | 6.02 (3.70-7.37) |
| route_obstacle | 342.97 (262.48-448.31) | 2.87 (1.00-8.00) | 37.98 (12.00-65.00) | 4.91 (2.14-6.04) |
