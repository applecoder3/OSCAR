#!/bin/sh
# Flash an ELF over SWD through the Black Magic Probe plugged into the Pi.
# PI=user@host overrides the Pi (default: the team Pi 5).
set -e
ELF=${1:?usage: flash.sh build/firmware.elf}
PI=${PI:-oscar@RaspberryPi5.local}
scp "$ELF" "$PI":/tmp/fw.elf
ssh "$PI" 'gdb-multiarch -nx --batch \
  -ex "target extended-remote /dev/ttyACM0" \
  -ex "monitor swd_scan" \
  -ex "attach 1" \
  -ex "load" \
  -ex "compare-sections" \
  -ex "detach" \
  /tmp/fw.elf'
