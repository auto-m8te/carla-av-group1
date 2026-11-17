# Lanebot: lane-following car in CARLA with ROS 2

A car that drives itself in the CARLA simulator on the Town04 motorway: it keeps
its lane, finds obstacles with its lidar, overtakes slower vehicles, and follows
German traffic rules (keep right, indicate before lane changes, 2-second gap,
speed signs, traffic lights, make way for blue lights). An instrument-cluster
window shows what the car senses and decides.

Course project, Autonomous Vehicles, SRH Berlin, WS 2026/27.

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy (`ros-jazzy-desktop-full`, `ros-dev-tools`)
- CARLA 0.9.16 in `~/carla-0.9.16` and its Python 3.12 client
- NVIDIA GPU (tested on a 6 GB RTX 1000 Ada)

Extra packages:

```bash
sudo apt install -y ros-jazzy-ackermann-msgs ros-jazzy-derived-object-msgs python3-transforms3d python3-pygame
```

## Setup

```bash
git clone <this repository> ~/lanebot
cd ~/lanebot && ./tools/setup_bridge.sh
rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install
```

`tools/setup_bridge.sh` downloads the CARLA ROS bridge and applies the four fixes
it needs for ROS 2 Jazzy and CARLA 0.9.16 (`patches/ros-bridge-jazzy.patch`).

## Run

```bash
./sim.sh demo        # fresh start, traffic, then the car drives (Ctrl+C stops it)
./sim.sh stop        # stop everything
```

| Command | What it does |
| --- | --- |
| `./sim.sh start` / `restart` / `stop` | CARLA, the bridge, one car and the camera viewer |
| `./sim.sh traffic` | Overtaking, slow, braking and oncoming cars, a lorry, ambulance and police |
| `./sim.sh obstacle 80` | Park a car 80 m ahead |
| `./sim.sh night` / `day` | Time of day (headlight dipping shows at night) |
| `./sim.sh drive` | Run all nodes and the instrument cluster |
| `./sim.sh status` | What is running |
| `./tools/run_test.sh 600 NAME` | 10-minute test with a rule checker; result in `logs/result_NAME.json` |

## Layout

| Path | Contents |
| --- | --- |
| `src/lanebot_ros/` | Our ROS 2 nodes: lane, obstacle, control, signals, audio, cluster, traffic |
| `src/lanebot_gz/` | Draft Gazebo version of the track (not used by the project) |
| `config/ego_vehicle.json` | The car's sensors |
| `patches/` | Our fixes to the CARLA ROS bridge |
| `tools/` | Bridge setup and the test runner / monitor |
| `sim.sh`, `gz.sh` | Start and stop everything (CARLA; Gazebo draft) |

## Known issue

When an ambulance comes up behind in our own lane, moving 0.7 m to the lane edge
is not always enough room, so the car can block it.
