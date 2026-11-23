#!/usr/bin/env bash
set -euo pipefail
# Execute ON THE PI HOST (not inside an existing container).
# Example: sudo bash create_uart_container.sh /dev/ttyAMA0
if (( $# != 1 )); then
  echo "Usage: sudo bash $0 /dev/ttyAMA0" >&2
  echo "First inspect: ls -l /dev/serial0; readlink -f /dev/serial0" >&2
  exit 2
fi
UART_REAL="$(readlink -f "$1")"
if [[ ! -c "$UART_REAL" ]]; then echo "ERROR: $UART_REAL is not a character device" >&2; exit 3; fi
if [[ ! -c /dev/ttyUSB0 ]]; then echo 'ERROR: LiDAR /dev/ttyUSB0 not present' >&2; exit 4; fi
if docker ps -a --format '{{.Names}}' | grep -qx oscar-ros-prototype; then
  echo 'Existing oscar-ros-prototype container; refusing to replace it.' >&2; exit 5
fi
if ! docker ps --format '{{.Names}}' | grep -qx oscar-ros-lidar; then
  echo 'Existing oscar-ros-lidar must be running to snapshot installed packages.' >&2; exit 6
fi
# Snapshot does not overwrite existing image or container.
docker commit oscar-ros-lidar oscar-jazzy-lidar:prototype
HOME_DIR="$(getent passwd oscar | cut -d: -f6)"
[[ -d "$HOME_DIR/oscar_ros" ]] || { echo 'Host bind directory missing'; exit 7; }
docker run -d --name oscar-ros-prototype --network host --restart unless-stopped \
  --device=/dev/ttyUSB0:/dev/ttyUSB0 \
  --device="$UART_REAL":/dev/oscar_pico \
  -v "$HOME_DIR/oscar_ros:/workspace" -w /workspace \
  oscar-jazzy-lidar:prototype sleep infinity
printf '\nCreated oscar-ros-prototype with Pico UART as /dev/oscar_pico.\n'
printf 'Original oscar-ros-lidar is unchanged. STOP one driver before starting another to avoid serial-port contention.\n'
