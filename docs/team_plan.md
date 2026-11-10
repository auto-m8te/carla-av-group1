# Group 1 — Project plan (E6 · Team roles, plan & architecture)

Course: Engineering Teamwork, WS2026/27, SRH Berlin (A. Laboudi)
Sprint 1: **10–24 Nov 2026** · Repository: `carla-av-group1` · Tracker: Trello board "Eng. Teamwrk"

## 1. Roles (as assigned on the Trello board)

| Member | Role | Epics | Report section |
|---|---|---|---|
| Prithiv | Simulation & integration, board owner | E1, E5 (integration) | — |
| Adil | Lane detection & steering | E2 | II. Literature review |
| Abir | Obstacle detection (sensors) | E3 | III. Methodology |
| Omar | Collision avoidance, testing, documentation | E4, E6 | I, V, VI |
| Khalid | City navigation & innovation (joined after kick-off) | E5 (navigation) | IV. Innovation |

Roles were a starting point, not a boundary: when a card was blocked or unstarted, another member picked it up so that the whole system could be integrated and tested. Section 6 records who actually built what.

## 2. Way of working

| What | Where | Rule |
|---|---|---|
| Tasks | Trello: one card per task, epic = label | a card moves to Done only with evidence attached (CSV, plot, video or PR) |
| Code | GitHub, one branch + pull request per card | tests must pass; large files (videos) go into Releases |
| Communication | WhatsApp group | blockers are posted the same day, not saved for the next meeting |
| Meetings | short stand-ups during the sprint, sprint review on 24 Nov | each person: done · next · blocked |

## 3. Sprint 1 goal and Definition of Done

**Goal:** the vehicle drives in CARLA and keeps its lane on the city map, with a working "obstacle ahead" signal.
(Reached and exceeded: the stop / avoid behaviour from the backlog epic E4 was also finished in Sprint 1.)

A card is **Done** when:
1. it runs on the team's CARLA machine (CARLA 0.9.16, Python 3.11, `requirements.txt`);
2. the code is in the shared repository and `python -m pytest -q` passes (no CARLA needed);
3. it is shown working in CARLA — a results CSV, a plot or a video is attached to the card;
4. the README explains how to run it.

## 4. Architecture

![architecture](../report/figures/architecture_owners.png)

| From | Value | Type |
|---|---|---|
| Lane module (camera) | `steering_angle` | float in [-1, 1] |
| Obstacle module (LiDAR) | `obstacle_distance` | metres to the nearest obstacle in the path, `None` if clear |
| Obstacle module (LiDAR) | `adjacent_lane_clear` | bool — the lane we would swerve into is free for the whole manoeuvre |
| Navigation | route steering, junction speed limit | route follower → bridge |
| Decision | `ControlCommand(throttle, steer, brake, mode)` | applied to the CARLA vehicle every tick |

Plug-in point: `CarlaAvoidanceBridge(world, ego, cfg, lane_steer_fn=..., obstacle_distance_fn=..., adjacent_clear_fn=..., route=...)`.

## 5. Status per epic (11 Nov 2026)

| Epic | Card(s) | Status | Evidence |
|---|---|---|---|
| E1 Simulation & tooling | CARLA setup, repo, README, interfaces, config | Done | repo, README, `AvoidanceConfig` |
| E2 Lane detection | camera lane detection (MVP), lane-keeping controller | Done | offline: 1,674 labelled frames, held-out test 91 % detection, 0.27 m offset error; closed loop camera + LiDAR 17/18 |
| E3 Obstacle detection | LiDAR obstacle distance + "ahead" flag, adjacent-lane check | Done | 200-run evaluation, distance error 0.3–0.5 m; noise test 15/15 |
| E4 Collision avoidance | stop vs. avoid logic, emergency stop + speed control, lane-change manoeuvre | Done | 130/130 static and curve runs, 0 collisions |
| E5 Navigation & integration | integrated pipeline, scenario testing, city navigation (junctions, routing) | Done | 200-run evaluation, moving traffic, route scenarios |
| E6 Management & docs | team plan, IEEE report, slides, demo video | Done (Section IV by Khalid to be reviewed) | report PDF, 14-slide deck, 3-min demo reel (release v1.0) |

## 6. Who made what

| Work | Epic | Card owner | Built by |
|---|---|---|---|
| Trello board, sprint plan, epics, module interface contract | E1/E6 | Prithiv | Prithiv |
| GitHub repository, README, configuration, CARLA setup on the team machine | E1 | Prithiv | Omar |
| Stop-or-avoid decision logic, emergency stop, speed control, lane-change manoeuvre, FOLLOW / ABORT modes | E4 | Omar | Omar |
| LiDAR obstacle detection (sensor wrapper, corridor, road removal, clustering, adjacent-lane and oncoming checks) | E3 | Abir | Omar, with Abir |
| Camera lane detection + lane-keeping steering, labelled-frame dataset and tuning | E2 | Adil | Omar |
| Scenario runner, curve / moving-traffic / route scenarios, analysis, plots, video recorder | E5 | Prithiv / unassigned | Omar |
| City navigation: route planner, route follower, junction speed policy | E5 | Khalid | Omar, with Khalid |
| Evaluations (Run A 90, Run B 200, camera, noise, navigation) and debugging in CARLA | E5 | — | Omar |
| Report Section I Introduction, V Findings, VI Conclusion, references, merging | E6 | Omar (I and VI shared) | Omar |
| Report Section II Literature review | E6 | Adil | Adil (obstacle-detection paragraph added by Omar) |
| Report Section III Methodology | E6 | Abir | Abir (draft); updated to the final implementation by Omar |
| Report Section IV Innovation | E6 | Khalid | drafted by Omar for Khalid to review |
| Architecture diagram, slides, demo video | E6 | Omar | Omar |

Tools: the code and documents were developed with AI coding assistants (Claude Code, OpenAI Codex) under Omar's direction; all results come from runs of the code in CARLA and are reproducible from the repository.

## 7. Git conventions

- Branch per card: `e2-lane-detection`, `e5-navigation`, …
- Commit messages: `feat:`, `fix:`, `test:`, `docs:` + short description.
- Pull request into `main`, tests must pass.
- Large files (videos, datasets) go into GitHub Releases or stay local, never into git.

Regenerate the architecture figure: `cd carla_av && python -m analysis.architecture_diagram --owners --out ../report/figures/architecture_owners.png`.
