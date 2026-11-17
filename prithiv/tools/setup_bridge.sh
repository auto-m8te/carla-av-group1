#!/bin/bash
# Download the CARLA ROS bridge at the version we tested and apply our Jazzy fixes:
#   1. accept CARLA 0.9.16 (version file)
#   2. numpy.bool -> bool (crashed when a car spawned)
#   3. do not reload an already loaded map (the reload crashed CARLA)
#   4. run the simulation at real time instead of as fast as possible
# and skip three optional packages that do not build on ROS 2 Jazzy.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE/src"
[ -d ros-bridge ] || git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git
cd ros-bridge
git checkout e9063d97ff5a724f76adbb1b852dc71da1dcfeec
git apply "$HERE/patches/ros-bridge-jazzy.patch"
touch pcl_recorder/COLCON_IGNORE rviz_carla_plugin/COLCON_IGNORE carla_ad_demo/COLCON_IGNORE
echo "ROS bridge ready. Now: cd $HERE && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install"
