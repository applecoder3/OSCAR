#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash
printf '\nROS packages:\n'
for pkg in oscar_preflight nav2_bringup ldlidar_stl_ros2 slam_toolbox; do
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then echo "PASS $pkg"; else echo "MISSING $pkg"; fi
done
printf '\nDevice status:\n'
ls -l /dev/ttyUSB0 || true
printf '\nActive topics:\n'
ros2 topic list
printf '\nSensor check (requires live LD19):\n'
ros2 topic hz /scan --window 10 || true
