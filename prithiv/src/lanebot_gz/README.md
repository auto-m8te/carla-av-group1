# lanebot_gz (draft)

A Gazebo Harmonic version of the lanebot test track, kept as a draft.
The project itself runs on CARLA 0.9.16 (`./sim.sh`).

It publishes the same topics as the CARLA bridge, so `lanebot_ros`'s
`obstacle_node` and `control_node` run on it unchanged.

Run it with `./gz.sh restart`, `./gz.sh obstacle 80`, `./gz.sh drive`
(stop CARLA first with `./sim.sh stop`).
