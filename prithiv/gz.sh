#!/usr/bin/env bash
# DRAFT, not used by the project: the team simulator is CARLA (see sim.sh).
# One-command control of the Gazebo version of the lanebot track.
#
#   ./gz.sh start         start Gazebo (with its 3D window), the ROS bridge and the car
#   ./gz.sh stop          stop everything
#   ./gz.sh restart       stop, then start (puts the car back at the start)
#   ./gz.sh obstacle 80   park a box 80 m ahead of the car
#   ./gz.sh drive         run lane, obstacle and control nodes together
#   ./gz.sh status        show what is running
#
# The obstacle and control nodes are the same ones you use with CARLA.
# Do not run this and ./sim.sh at the same time: they use the same topic names.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGS="$HERE/logs"
WORLD="$HERE/src/lanebot_gz/worlds/lanebot.sdf"

cd "$HERE"
mkdir -p "$LOGS"
source /opt/ros/jazzy/setup.bash
[ -f install/setup.bash ] && source install/setup.bash
# Render on the NVIDIA card rather than the Intel one.
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia

gz_running()  { pgrep -f '[g]z sim.*lanebot.sdf' >/dev/null; }
stop_nodes()  { pkill -INT -f '[l]ib/lanebot_ros/|[l]ib/lanebot_gz/lane_node' 2>/dev/null; }
build()       { colcon build --symlink-install --packages-select lanebot_ros lanebot_gz | tail -1; source install/setup.bash; }

start() {
  if gz_running; then echo "Already running. Use './gz.sh restart' for a fresh start."; exit 1; fi
  if pgrep -f '[C]arlaUE4-Linux-Shipping' >/dev/null; then
    echo "CARLA is running. Stop it first with './sim.sh stop'."; exit 1
  fi

  echo "[1/4] Building..."
  build
  ros2 run lanebot_gz make_world "$WORLD" >/dev/null

  echo "[2/4] Starting Gazebo..."
  if [ -n "$HEADLESS" ]; then
    setsid nohup gz sim -s -r --headless-rendering "$WORLD" > "$LOGS/gazebo.log" 2>&1 &
  else
    setsid nohup gz sim -r "$WORLD" > "$LOGS/gazebo.log" 2>&1 &
  fi
  for _ in $(seq 60); do gz topic -l 2>/dev/null | grep -q '^/lidar/points' && break; sleep 1; done

  echo "[3/4] Connecting Gazebo to ROS..."
  # '[' = Gazebo to ROS, ']' = ROS to Gazebo.  The -r remaps give the
  # topics the same names as in CARLA.
  setsid nohup ros2 run ros_gz_bridge parameter_bridge \
    /model/ego/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
    /model/ego/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry \
    /lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
    /rgb_view/image@sensor_msgs/msg/Image[gz.msgs.Image \
    /rgb_front/image@sensor_msgs/msg/Image[gz.msgs.Image \
    --ros-args \
    -r /model/ego/odometry:=/carla/ego_vehicle/odometry \
    -r /lidar/points:=/carla/ego_vehicle/lidar \
    -r /rgb_view/image:=/carla/ego_vehicle/rgb_view/image \
    -r /rgb_front/image:=/carla/ego_vehicle/rgb_front/image \
    > "$LOGS/gz_bridge.log" 2>&1 &

  echo "[4/4] Starting the car..."
  setsid nohup ros2 run lanebot_gz vehicle_node > "$LOGS/vehicle.log" 2>&1 &
  if [ -z "$HEADLESS" ]; then
    setsid nohup ros2 run rqt_image_view rqt_image_view /carla/ego_vehicle/rgb_view/image \
      > "$LOGS/viewer.log" 2>&1 &
  fi
  sleep 3
  echo "Ready. Next:  ./gz.sh obstacle 80   then   ./gz.sh drive"
}

stop() {
  stop_nodes
  pkill -INT -f '[l]ib/rqt_image_view/' 2>/dev/null
  pkill -f '[l]ib/lanebot_gz/vehicle_node|[p]arameter_bridge /model/ego' 2>/dev/null
  pkill -f '[g]z sim.*lanebot.sdf' 2>/dev/null
  for _ in $(seq 8); do gz_running || break; sleep 1; done
  pkill -9 -f '[g]z sim.*lanebot.sdf' 2>/dev/null
  echo "Stopped."
}

case "${1:-}" in
  start)   start ;;
  stop)    stop ;;
  restart) stop; start ;;
  obstacle)
    ros2 run lanebot_gz spawn_obstacle "${2:-80}"
    ;;
  drive)
    build
    stop_nodes; sleep 1
    ros2 run lanebot_gz lane_node      > "$LOGS/lane.log" 2>&1 &
    ros2 run lanebot_ros obstacle_node > "$LOGS/obstacle.log" 2>&1 &
    trap 'stop_nodes' EXIT
    sleep 2
    ros2 run lanebot_ros control_node
    ;;
  status)
    gz_running && echo "Gazebo:     running" || echo "Gazebo:     stopped"
    pgrep -f '[p]arameter_bridge /model/ego' >/dev/null && echo "ROS bridge: running" || echo "ROS bridge: stopped"
    pgrep -f '[l]ib/lanebot_gz/vehicle_node' >/dev/null && echo "Car:        running" || echo "Car:        stopped"
    ;;
  *)       sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//' ;;
esac
