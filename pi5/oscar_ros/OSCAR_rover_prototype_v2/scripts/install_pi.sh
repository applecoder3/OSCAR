#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS=/workspace/ros2_ws
mkdir -p "$WS/src"
if [[ -e "$WS/src/oscar_preflight" && ! -L "$WS/src/oscar_preflight" ]]; then
  echo "ERROR: Existing oscar_preflight is not a symlink. Refusing to overwrite." >&2
  exit 2
fi
if [[ -L "$WS/src/oscar_preflight" ]]; then
  if [[ "$(readlink -f "$WS/src/oscar_preflight")" != "$(readlink -f "$ROOT/oscar_preflight")" ]]; then
    echo "ERROR: oscar_preflight symlink points elsewhere. Refusing to replace." >&2
    exit 3
  fi
else
  ln -s "$ROOT/oscar_preflight" "$WS/src/oscar_preflight"
fi
cd "$WS"
colcon build --packages-select oscar_preflight --symlink-install --parallel-workers 2
source "$WS/install/setup.bash"
python3 "$ROOT/scripts/prepare_nav2.py" /opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml "$ROOT/config/nav2_rover.yaml"
ros2 pkg executables oscar_preflight
printf '\nINSTALL COMPLETE: Motors remain disabled by default.\n'
