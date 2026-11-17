#!/usr/bin/env bash
# One-command control of the CARLA 0.9.16 + ROS 2 Jazzy simulation (no Docker).
#
#   ./sim.sh start        start CARLA, the bridge, one car and the viewer
#   ./sim.sh stop         stop everything
#   ./sim.sh restart      stop, then start (puts the car back at the start)
#   ./sim.sh obstacle 80  park a car 80 m ahead of yours
#   ./sim.sh drive        drive: lane, obstacle, control, lights, sound, cluster screen
#   ./sim.sh demo         everything: fresh start, traffic, then drive
#   ./sim.sh traffic      add traffic: overtaking cars, a slow car, oncoming cars,
#                         ambulance and police (runs in the background)
#   ./sim.sh night        switch to night (headlights matter)   ./sim.sh day  back to day
#   ./sim.sh run NODE     build the package and run one of your nodes
#   ./sim.sh status       show what is running

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CARLA_DIR="$HOME/carla-0.9.16"
LOGS="$HERE/logs"
TOWN="Town04"
QUALITY="Low"         # CARLA graphics: Low or Epic. Epic is too heavy for this laptop (2 fps)
FPS=30                # simulation frames per second (also the camera frame rate)
EXTRA=""             # extra CARLA graphics settings (Unreal console commands)
DELTA=$(awk "BEGIN{printf \"%.4f\", 1/$FPS}")
# Start pose in ROS coordinates: x,y,z,roll,pitch,yaw (right-hand lane of the motorway: keep right, StVO §2)
SPAWN="-483.7,-242.9,0.3,0,0,90.8"

cd "$HERE"
mkdir -p "$LOGS"
source /opt/ros/jazzy/setup.bash
export PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0   # logs appear in the files straight away
[ -f install/setup.bash ] && source install/setup.bash

carla_running()  { pgrep -f '[C]arlaUE4-Linux-Shipping' >/dev/null; }
bridge_running() { pgrep -f '[l]ib/carla_ros_bridge/bridge' >/dev/null; }
build()          { colcon build --symlink-install --packages-select lanebot_ros | tail -1; source install/setup.bash; }
stop_nodes()     { pkill -INT -f '[l]ib/lanebot_ros/(lane|obstacle|control|signals|audio|cluster)_node' 2>/dev/null; }

start() {
  if carla_running || bridge_running; then
    echo "Already running. Use './sim.sh restart' for a fresh start."; exit 1
  fi

  # CARLA sometimes crashes while starting up; try up to three times.
  for attempt in 1 2 3; do
    echo "[1/5] Starting CARLA server on the NVIDIA GPU ($QUALITY quality, $FPS fps)..."
    ( cd "$CARLA_DIR" && setsid nohup nice -n 10 ./CarlaUE4.sh "/Game/Carla/Maps/$TOWN" -RenderOffScreen -quality-level=$QUALITY ${EXTRA:+-ExecCmds="$EXTRA"} -nosound -prefernvidia \
        > "$LOGS/carla.log" 2>&1 & )
    for _ in $(seq 60); do ss -ltn | grep -q ':2000 ' && break; sleep 1; done
    sleep 8

    echo "[2/5] Starting ROS bridge on $TOWN..."
    setsid nohup ros2 launch carla_ros_bridge carla_ros_bridge.launch.py town:=$TOWN timeout:=60 fixed_delta_seconds:=$DELTA \
      > "$LOGS/bridge.log" 2>&1 &
    ok=""
    for _ in $(seq 90); do
      grep -q 'Created' "$LOGS/bridge.log" 2>/dev/null && { ok=1; break; }
      carla_running || break
      sleep 1
    done
    [ -n "$ok" ] && carla_running && break
    echo "CARLA stopped while starting (attempt $attempt of 3); trying again..."
    stop > /dev/null
    [ "$attempt" = 3 ] && { echo "CARLA would not start. See logs/carla.log."; exit 1; }
  done

  echo "[3/5] Building your package..."
  build

  echo "[4/5] Spawning one car..."
  setsid nohup ros2 launch carla_spawn_objects carla_example_ego_vehicle.launch.py \
    spawn_point_ego_vehicle:=$SPAWN objects_definition_file:=$HERE/config/ego_vehicle.json > "$LOGS/spawn.log" 2>&1 &
  for _ in $(seq 60); do grep -q 'All objects spawned' "$LOGS/spawn.log" 2>/dev/null && break; sleep 1; done

  echo "[5/5] Opening the camera viewer..."
  setsid nohup ros2 run rqt_image_view rqt_image_view /carla/ego_vehicle/rgb_view/image \
    > "$LOGS/viewer.log" 2>&1 &
  echo "Ready. Next:  ./sim.sh obstacle 80   then   ./sim.sh drive"
}

stop() {
  stop_nodes
  pkill -INT -f '[l]ib/lanebot_ros/traffic_node' 2>/dev/null && sleep 2   # removes its cars
  pkill -INT -f '[l]ib/rqt_image_view/' 2>/dev/null
  pkill -INT -f '[r]os2 launch carla_spawn_objects' 2>/dev/null
  sleep 2
  pkill -INT -f '[r]os2 launch carla_ros_bridge' 2>/dev/null
  sleep 3
  pkill -9 -f '[l]ib/carla_spawn_objects/|[l]ib/carla_ros_bridge/' 2>/dev/null
  # CARLA ignores the first polite stop signal, so ask, wait, then force it.
  pkill -f '[C]arlaUE4-Linux-Shipping' 2>/dev/null
  for _ in $(seq 8); do carla_running || break; sleep 1; done
  pkill -9 -f '[C]arlaUE4-Linux-Shipping' 2>/dev/null
  sleep 1
  echo "Stopped."
}

case "${1:-}" in
  start)   start ;;
  stop)    stop ;;
  restart) stop; start ;;
  obstacle)
    ros2 run lanebot_ros spawn_obstacle "${2:-80}"
    ;;
  drive)
    build
    stop_nodes; sleep 1
    ros2 run lanebot_ros lane_node     > "$LOGS/lane.log" 2>&1 &
    ros2 run lanebot_ros obstacle_node > "$LOGS/obstacle.log" 2>&1 &
    ros2 run lanebot_ros signals_node  > "$LOGS/signals.log" 2>&1 &
    ros2 run lanebot_ros audio_node    > "$LOGS/audio.log" 2>&1 &
    ros2 run lanebot_ros cluster_node  > "$LOGS/cluster.log" 2>&1 &
    trap 'stop_nodes' EXIT
    sleep 2
    ros2 run lanebot_ros control_node
    ;;
  traffic)
    pkill -INT -f '[l]ib/lanebot_ros/traffic_node' 2>/dev/null; sleep 1
    setsid nohup ros2 run lanebot_ros traffic_node > "$LOGS/traffic.log" 2>&1 &
    echo "Traffic started (log: logs/traffic.log). It stops with './sim.sh stop'."
    ;;
  demo)
    "$0" restart && "$0" traffic && exec "$0" drive
    ;;
  night|day)
    python3 -c "import carla, sys; w = carla.Client('localhost', 2000).get_world(); w.set_weather(carla.WeatherParameters.ClearNight if sys.argv[1] == 'night' else carla.WeatherParameters.ClearNoon)" "$1" && echo "It is now $1."
    ;;
  run)
    shift
    [ $# -ge 1 ] || { echo "Usage: ./sim.sh run NODE_NAME"; exit 1; }
    build
    ros2 run lanebot_ros "$@"
    ;;
  status)
    carla_running  && echo "CARLA server: running" || echo "CARLA server: stopped"
    bridge_running && echo "ROS bridge:   running" || echo "ROS bridge:   stopped"
    ros2 topic info /carla/ego_vehicle/rgb_view/image 2>/dev/null | sed -n 2p | sed 's/Publisher count/Cars in the simulation/'
    nvidia-smi --query-gpu=memory.used --format=csv,noheader | sed 's/^/GPU memory in use: /'
    ;;
  *)       sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//' ;;
esac
