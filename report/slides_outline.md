# Group 1 presentation — speaker notes

## 1. Autonomous driving in CARLA

We are Group 1 at SRH Berlin, presenting our WS2026/27 project for Prof. A. Laboudi. Our vehicle follows a lane, detects obstacles, and chooses whether to stop or steer around them. The implementation runs in CARLA 0.9.16. We will explain the modules, the evaluation, and the limitations of the current system.

## 2. Problem & goals

The task combines perception, decision making, and vehicle control in a simulated city. The first requirement is to follow a driving lane. The second is to detect obstacles, and the third is to avoid collisions by stopping or steering around them. The fourth is safe navigation, with junction navigation still part of the future work.

## 3. Team & roles

Prithiv owns simulation and integration, while Adil owns lane detection and steering. Abir develops the LiDAR obstacle detector. Omar owns collision avoidance, testing, and documentation. Khalid joined later and contributes city navigation and innovation.

## 4. Architecture

The architecture separates sensing, obstacle detection, decision making, and control, with ownership shown in the diagram. The obstacle interface supplies distance and adjacent-lane availability to the decision module. The diagram also shows the intended camera lane branch, which is still pending integration. In the reported evaluations, lane steering and the LiDAR corridor use map information.

## 5. Obstacle detection

A roof-mounted, 32-channel LiDAR supplies one sweep per tick. A range gate first filters the points, then a lane-shaped corridor from the map selects the relevant path. Road removal uses grade-limited height estimates in 2 m bins. A cluster must contain at least 3 points within 1 m to count as an obstacle.

## 6. Decision logic

The controller uses seven modes, shown as the normal stop and avoidance branches. The stopping threshold combines reaction distance, braking distance, and a margin. Avoidance is allowed only if the adjacent lane is free over 60 m. An obstacle below 5 m triggers emergency braking in any mode, and braking preserves the manoeuvre state when needed.

## 7. Test design

We test across six towns with three scenarios and five seeded runs for each combination. The scenarios are an empty lane, an obstacle with a blocked adjacent lane, and an obstacle with a free adjacent lane. The same seed set is evaluated once with ground-truth obstacle perception and once with LiDAR. Lane steering remains based on map ground truth in both evaluations.

## 8. Results

Both perception configurations completed all 90 runs successfully with no collisions. LiDAR distance MAE was 0.48 m for stopping and 0.44 m for avoidance. False alarms occurred on 136 of 23,715 ticks, or 0.6 percent, and none changed vehicle behaviour. These outcomes apply to the tested static-obstacle scenarios with ground-truth lane steering.

## 9. Example runs

These plots illustrate the ground-truth evaluation runs rather than the LiDAR distance-error measurements. Town01 demonstrates an avoidance manoeuvre through the empty oncoming lane. Town03 demonstrates braking and holding stopped behind an obstacle. Town04 demonstrates avoidance on a multi-lane highway, with the manoeuvre returning to the original lane.

## 10. What broke and how we fixed it

CARLA testing revealed twelve problems that were fixed during development; this slide selects four. Road and lane identifiers changed between segments, so adjacency checks moved to geometry. LiDAR guardrail detections were reduced by following the lane shape, and elevated-road errors were addressed by estimating road height near the path only. Limiting the corridor to the end of the path reduced false-alarm ticks from 1,564 to 136.

## 11. Limitations & next steps

Lane steering still uses ground truth, and Adil's camera lane module is the next integration step. The current experiments use static obstacles, so moving traffic requires time-to-collision reasoning. The LiDAR is noise-free, and steep crests in Town05 can still produce false alarms. Further work includes sensor noise and city navigation through junctions, with Khalid contributing to navigation.

## 12. Demo / Q&A

The recorded demonstration is available in the GitHub release tagged v0.1-demo. The command shown runs a Town04 demonstration with LiDAR obstacle and adjacent-lane detection, a HUD, and real-time pacing. It is provided for use with a running CARLA environment; building this presentation does not start CARLA. We welcome questions about detection, decision logic, testing, and remaining integration work.

All requested facts placed in slides or speaker notes.
