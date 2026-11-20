#!/bin/sh
set -e
ELF=${1:?usage: flash.sh build/firmware.elf}
scp "$ELF" tripleberryjam@pi-zero-2w.local:/tmp/fw.elf
ssh tripleberryjam@pi-zero-2w.local 'gdb-multiarch -nx --batch \
  -ex "target extended-remote /dev/ttyACM0" \
  -ex "monitor swd_scan" \
  -ex "attach 1" \
  -ex "load" \
  -ex "compare-sections" \
  -ex "detach" \
  /tmp/fw.elf'
