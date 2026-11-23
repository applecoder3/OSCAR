#!/usr/bin/env bash
set -euo pipefail
# This script must run ON the WSL machine, against an independent Ethan firmware worktree.
if (( $# != 1 )); then echo "Usage: $0 ~/projects/OSCAR_pico_velocity" >&2; exit 2; fi
TREE="$(realpath "$1")"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "$TREE/firmware/src/diag.c" || ! -f "$TREE/firmware/inc/oscar_hw.h" || ! -f "$TREE/firmware/CMakeLists.txt" ]]; then
  echo "ERROR: Provided directory is not Ethan-based OSCAR firmware worktree" >&2; exit 3
fi
if grep -q '^add_executable(oscar_autonomous ' "$TREE/firmware/CMakeLists.txt"; then
  echo 'Autonomous target already present; no changes applied.'; exit 0
fi
if [[ -e "$TREE/firmware/src/oscar_autonomous.c" ]]; then
  echo 'ERROR: Existing oscar_autonomous.c would be overwritten.' >&2; exit 4
fi
cp "$ROOT/firmware/oscar_autonomous.c" "$TREE/firmware/src/oscar_autonomous.c"
cat >> "$TREE/firmware/CMakeLists.txt" <<'CMAKE'

# Independent OSCAR_V1 UART autonomous prototype. Motor outputs DISABLED by default.
add_executable(oscar_autonomous src/diag.c src/oscar_autonomous.c)
pico_generate_pio_header(oscar_autonomous ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)
target_include_directories(oscar_autonomous PRIVATE ${CMAKE_CURRENT_LIST_DIR}/inc)
target_link_libraries(oscar_autonomous pico_stdlib hardware_pwm hardware_adc hardware_pio m)
pico_enable_stdio_usb(oscar_autonomous 0)
pico_enable_stdio_uart(oscar_autonomous 1)
pico_add_extra_outputs(oscar_autonomous)
CMAKE
printf 'Added independent oscar_autonomous target to %s\n' "$TREE"
printf 'Build from firmware/: cmake -S . -B build && cmake --build build --target oscar_autonomous -j2\n'
