#!/usr/bin/env bash
# Copy only rover_sim; do not overwrite the user's working rover_description.
set -e
source /opt/ros/lyrical/setup.bash
source_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HOME/rover_ws/src"
mkdir -p "$HOME/rover_ws/package_backups"
touch "$HOME/rover_ws/package_backups/COLCON_IGNORE"
if [ -d "$HOME/rover_ws/src/rover_sim" ]; then
    backup_dir="$HOME/rover_ws/package_backups/rover_sim_$(date +%Y%m%d_%H%M%S)_$$"
    mkdir -p "$backup_dir"
    cp -a "$HOME/rover_ws/src/rover_sim/." "$backup_dir/"
    printf 'Existing rover_sim backed up to %s\n' "$backup_dir"
fi
cp -a "$source_dir/rover_sim/." "$HOME/rover_ws/src/rover_sim/"
chmod +x "$HOME/rover_ws/src/rover_sim/scripts/command_guard.py" "$HOME/rover_ws/src/rover_sim/scripts/keyboard.py"
cd "$HOME/rover_ws"
colcon build --symlink-install --packages-select rover_sim
