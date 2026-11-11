# Group 1 presentation — speaker notes

## 1. Autonomous driving in CARLA

We are Group 1 at SRH Berlin, presenting our WS2026/27 project for Prof. A. Laboudi. Our vehicle follows a lane, detects obstacles, and chooses whether to stop or steer around them. The implementation runs in CARLA 0.9.16. We will explain the modules, the evaluation, and the limitations of the current system.

## 2. Problem & goals

The task combines perception, decision making, and vehicle control in a simulated city. The first requirement is to follow a driving lane. The second is to detect obstacles, and the third is to avoid collisions by stopping or steering around them. The fourth is safe navigation, including implemented route navigation through junctions.

## 3. Team & roles

Prithiv owns simulation and integration, while Adil owns lane detection and steering. Abir develops the LiDAR obstacle detector. Omar owns collision avoidance, testing, and documentation. Khalid joined later and contributes city navigation and innovation.

## 4. Architecture

The architecture separates sensing, obstacle detection, decision making, and control, with ownership shown in the diagram. The obstacle interface supplies distance and adjacent-lane availability to the decision module. The diagram also shows the camera lane branch, which is evaluated separately in Run C. In the reported evaluations, lane steering and the LiDAR corridor use map information.

## 5. Obstacle detection

A roof-mounted, 32-channel LiDAR supplies one sweep per tick. A range gate first filters the points, then a lane-shaped corridor from the map selects the relevant path. Road removal uses grade-limited height estimates in 2 m bins. A cluster must contain at least 3 points within 1 m and span at least 0.25 m vertically to count as an obstacle.

## 6. Decision logic

The controller includes FOLLOW and ABORT alongside the modes, shown as the normal stop and avoidance branches. The stopping threshold combines reaction distance, braking distance, and a margin. Avoidance requires projected adjacent clearance and a 1 s debounce. Moving scenarios extend sensing to 180 m. An obstacle below 5 m triggers emergency braking in any mode, and braking preserves the manoeuvre state when needed.

## 7. Test design

Run A preserves 90 original ground-truth runs. Final Run B uses LiDAR obstacle and adjacent-lane detection with map steering. Core scenarios have 30 runs each, curve scenarios 20 each, lead scenarios 25 each and oncoming overtakes 20. Some town/scenario combinations are unavailable. Run C is a separate 18-run camera-steering evaluation; noise testing is a separate 15-run Town04 experiment.

## 8. Results

Final Run B achieved 194/200 successes and zero collisions with map-waypoint steering. Four lead-brake runs did not fully stop within the time limit despite staying about 5–6 m behind. Two Town02_Opt oncoming runs failed. Run 2 has waited_for_oncoming=True; run 4 has False, contradicting a claim that both remained safe. Zero collisions does not establish safe lane use.

## 9. Moving traffic

Gap tracking estimates obstacle speed. FOLLOW uses a desired gap of 6 m plus 1.8 s times ego speed and brakes below 3 s TTC. Possibly moving obstacles cannot be overtaken, including before motion confirmation. Adjacent clearance must remain free for 1 s; unsafe committed pull-outs ABORT back behind the obstacle when room allows. Low-speed pull-out control supports restarting after waiting. Moving scenarios use 64-channel 185 m LiDAR because an overtake needs approximately 150 m of visibly free lane.

## 10. Camera lane detection

The pipeline combines local-contrast white and yellow masks with curb edges, a wider ROI, boundary-pair checks and a lane-width prior. PD steering uses median filtering and a rate limit. Tuning used drives 0–1 of 1,674 labelled CARLA frames; held-out drive 2 has 558 frames. Detection is 91.2%, offset MAE 0.265 m rounded to 0.27 m, and steering-sign agreement 73.2%. MAE uses detected frames and sign metrics use absolute true offset above 0.2 m. Town01 and Town10HD remain weak. Run C achieved 17/18, with one collision after Town03_Opt empty_lane run 1 drifted off the road.

## 11. Example runs

All three plots are from final Run B. The curve-stop example shows braking on curved geometry. The lead-vehicle example shows FOLLOW. The oncoming example shows waiting before completing the pass and return. These runs use map steering, not camera steering.

## 12. City navigation

Run D: six towns, five runs per scenario per town. Directed topology graph with Dijkstra and a 30 m parallel-lane-change penalty outside junctions. Pure pursuit uses 2 m + 0.5 s times speed, clamped 3–6 m. Junction and sharp-turn limits are 5 and 4 m/s. The route corridor is cut where it turns away; no overtaking within 25 m of junctions. route_obstacle passed 30/30; route passed 18/30 under the strict offset criterion below 1.5 m outside junctions. 29/30 routes reached the goal, inferred from duration below 90 s; only Town05_Opt run 4 timed out. Nine failures were 1.5–2.1 m corner cutting and two Town04 failures were about 5.2 m during planned lane changes. Zero collisions in 60 runs. Detector regression: Town01 and Town04, empty/stop/avoid, 30/30 and zero collisions.

## 13. What broke and how we fixed it

The LiDAR corridor originally swept onto guardrails; it now follows map lane geometry. Stale sensor frames delayed braking under load; bounded current-frame reads fix this. Oncoming cars at 60–80 m were filtered as road after empty height bins; retaining the last height and extending LiDAR range supports early detection. Standard maps crashed the CARLA renderer, so Town02, Town03 and Town05 use layered _Opt variants in final Run B.

## 14. Limitations & next steps

Run B includes selected moving traffic, but not pedestrians or dense traffic. Map steering and map corridor geometry remain dependencies. Camera testing is small and town-dependent. Noise testing is limited to one town and range-noise level; it achieved 15/15, with obstacle-distance MAE about 1.1 m. Run D still shows pure-pursuit corner cutting at junction exits. Junctions have no traffic-light or right-of-way handling, and routes were tested without other traffic apart from a parked obstacle. The final matrix uses _Opt maps for renderer compatibility.

## 15. Demo / Q&A

The recorded demo is in the GitHub release tagged v0.1-demo. It predates the final results. Building these documents does not start CARLA. Final results: Run B 194/200 with zero collisions; Run C 17/18 with one collision. Questions can address detection, control, test coverage and the remaining camera and navigation work.

All requested facts placed in slides or speaker notes.
