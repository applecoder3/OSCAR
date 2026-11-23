#!/usr/bin/env bash
# Pi 5 -> Pico link check through the team's serial_bridge (oscar_preflight).
# Proves ROS /cmd_vel -> serial_bridge -> /dev/ttyAMA0 -> oscar_autonomous ->
# wheels, plus the handshake, encoder telemetry, stale-command disarm and E-stop.
#
# WHEELS OFF THE GROUND. Pico flashed with oscar_autonomous, motors-on build.
# Nothing else may have /dev/ttyAMA0 open (quit screen first: Ctrl-a k y).
#
# Run on the Pi 5, one terminal:
#   sudo docker exec -i oscar-ros-lidar bash < ~/bridge_check.sh
#
# Uses its own ROS domain so it can't see or drive anything else that's running.
# Scan/odom guards are off because this is a wheels-up link test, not navigation.

export ROS_DOMAIN_ID=77
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash
set -u # after the ROS setup scripts, which read unset variables

PIDS=()
# SIGINT so the bridge's shutdown path runs (it sends DISARM before closing).
cleanup() {
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done
  wait 2>/dev/null
  pkill -INT -f 'lib/oscar_preflight/serial_bridge' 2>/dev/null
}
trap cleanup EXIT

say() { printf '\n=== %s\n' "$*"; }
status() { timeout 5 ros2 topic echo --once /pico/status 2>/dev/null | grep -o 'firmware_ready.*' | tr -d "'"; }

say "pyserial in the container"
if ! python3 -c 'import serial' 2>/dev/null; then
  echo "FAIL: python3 'serial' module missing. Install with: apt-get install -y python3-serial"
  exit 1
fi
echo ok

say "start serial_bridge on /dev/ttyAMA0"
# Run the entry point directly (not via ros2 run) so the PID is the bridge itself.
/workspace/ros2_ws/install/oscar_preflight/lib/oscar_preflight/serial_bridge --ros-args \
  -p enable_hardware:=true -p port:=/dev/ttyAMA0 \
  -p enable_scan_guard:=false -p enable_odom_guard:=false >/tmp/bridge.log 2>&1 &
PIDS+=($!)

ready=""
for i in $(seq 10); do
  s=$(status)
  echo "$s"
  case "$s" in *firmware_ready=True*) ready=1; break ;; esac
done
if [ -z "$ready" ]; then
  echo "FAIL: no HELLO OSCAR_V1 from the Pico. Check: oscar_autonomous flashed, screen closed, TX/RX wiring."
  tail -20 /tmp/bridge.log
  exit 1
fi
echo "PASS: handshake"

say "encoder telemetry"
enc=$(timeout 5 ros2 topic echo --once /pico/telemetry_raw 2>/dev/null | grep -o 'ENC.*' | tr -d "'")
if [ -n "$enc" ]; then echo "$enc"; echo "PASS: telemetry"; else echo "FAIL: no ENC lines"; fi

# One drive phase: stream /cmd_vel, arm, watch, then stop streaming and check
# that the bridge disarms on its own.
drive() { # label vx wz
  say "$1 for 2 s  (vx=$2 wz=$3)"
  ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: $2}, angular: {z: $3}}" >/dev/null 2>&1 &
  local pub=$!
  sleep 2
  ros2 topic pub --once -w 1 /pico/arm std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1
  sleep 1
  local s; s=$(status); echo "$s"
  case "$s" in *drive_ok=True*) echo "PASS: driving -- look at the wheels" ;;
               *) echo "FAIL: not driving" ;; esac
  sleep 1
  kill "$pub" 2>/dev/null; wait "$pub" 2>/dev/null
  sleep 1
  s=$(status); echo "$s"
  case "$s" in *armed=False*) echo "PASS: /cmd_vel stopped -> disarmed" ;;
               *) echo "FAIL: still armed after /cmd_vel stopped" ;; esac
}

drive "FORWARD: all four wheels roll forward" 0.05 0.0
drive "TURN LEFT: left wheels back, right wheels forward" 0.0 0.3

say "E-STOP while driving"
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}" >/dev/null 2>&1 &
pub=$!
sleep 2
ros2 topic pub --once -w 1 /pico/arm std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1
sleep 1
ros2 topic pub --once -w 1 /pico/estop std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1
sleep 0.5
s=$(status); echo "$s"
case "$s" in *armed=False*estop=True*) echo "PASS: E-stop stopped the wheels" ;;
             *) echo "FAIL: E-stop" ;; esac
kill "$pub" 2>/dev/null

say "bridge log (last lines)"
tail -8 /tmp/bridge.log
say "done -- bridge stopped, Pico left DISARMED"
