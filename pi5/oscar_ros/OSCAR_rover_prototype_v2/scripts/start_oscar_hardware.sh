#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash

PROJECT=/workspace/OSCAR_rover_prototype_v2
PIDS=()

cleanup() {
    echo "Stopping OSCAR..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

start_node() {
    echo "Starting $1"
    shift
    "$@" &
    PIDS+=("$!")
    sleep 3
}

start_node "LD19 LiDAR" \
    ros2 launch ldlidar_stl_ros2 ld19.launch.py

start_node "Pico serial bridge (disarmed)" \
    ros2 run oscar_preflight serial_bridge \
    --ros-args -p enable_hardware:=${OSCAR_HARDWARE_ENABLED:-false} -p port:=/dev/ttyAMA0 -p baud:=115200

start_node "Real rear encoder odometry" \
    ros2 run oscar_preflight rear_odometry

start_node "SLAM" \
    ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="$PROJECT/config/slam_rover.yaml"

start_node "Nav2" \
    ros2 launch nav2_bringup navigation_launch.py \
    params_file:="$PROJECT/config/nav2_rover.yaml" \
    use_sim_time:=false autostart:=true use_composition:=False

start_node "YOLO detector" \
    /workspace/yolo_env/bin/python \
    "$PROJECT/onnx_ros_detector.py"

start_node "Outlet targeting (navigation disabled)" \
    ros2 run oscar_preflight outlet_approach \
    --ros-args -p enable_navigation_goals:=false

echo "OSCAR startup complete."
echo "HARDWARE PREPARATION MODE: real odometry; physical serial disabled."
echo "Press Ctrl+C to stop."

wait
