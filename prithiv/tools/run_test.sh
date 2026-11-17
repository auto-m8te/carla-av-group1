#!/bin/bash
# 10-minute (or any length) test: fresh start, traffic, drive, and a monitor that
# fails the run on collisions, near misses, leaving the lane/road, speeding,
# red-light crossings or crashed programs. Result: logs/result_NAME.json
# usage: run_test.sh SECONDS NAME
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; cd "$HERE/.."; source /opt/ros/jazzy/setup.bash
./sim.sh restart > logs/start.out 2>&1; tail -1 logs/start.out
source install/setup.bash
./sim.sh traffic > /dev/null
timeout -s INT $(( $1 + 15 )) ./sim.sh drive > logs/drive_$2.out 2>&1 < /dev/null &
sleep 6
python3 $HERE/monitor.py $1 logs/result_$2.json
sleep 12
grep -E -- "->" logs/drive_$2.out | cut -d: -f2- | sort | uniq -c | sort -rn | head -12
grep -E "INFO|WARN" logs/traffic.log | cut -d']' -f3- | sort | uniq -c | head -12
for f in lane obstacle control signals audio cluster; do grep -l "Traceback" logs/$f.log 2>/dev/null && grep -B2 -A8 Traceback logs/$f.log | grep -vE "KeyboardInterrupt|ExternalShutdown|rclpy/executors|wait_for_ready|_spin_once|spin_once|rclpy.spin" | head -12; done
