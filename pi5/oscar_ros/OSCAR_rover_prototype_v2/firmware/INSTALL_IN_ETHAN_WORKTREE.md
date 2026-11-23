# Pico firmware integration (Ethan's physical hardware)

This autonomous firmware is a **separate target**, not a replacement for the existing diagnostic `firmware` target. It uses Ethan's actual `oscar_hw.h`, `diag.c` and `quadrature_encoder.pio` on branch `fw/bringup-tests` (observed November 20). Do not overwrite his changes.

Copy `oscar_autonomous.c` into `firmware/src/` in an isolated worktree that already contains Ethan's `firmware/` directory. Append to **that worktree's** `firmware/CMakeLists.txt`:

```cmake
add_executable(oscar_autonomous src/diag.c src/oscar_autonomous.c)
pico_generate_pio_header(oscar_autonomous ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)
target_include_directories(oscar_autonomous PRIVATE ${CMAKE_CURRENT_LIST_DIR}/inc)
target_link_libraries(oscar_autonomous pico_stdlib hardware_pwm hardware_adc hardware_pio m)
pico_enable_stdio_usb(oscar_autonomous 0)
pico_enable_stdio_uart(oscar_autonomous 1)
pico_add_extra_outputs(oscar_autonomous)
```

**Default `OSCAR_ENABLE_MOTOR_OUTPUT=0`: builds and parses commands but no PWM motor drive.** After physical validation with wheels off the ground, deliberately enable output only for this target by adding `target_compile_definitions(oscar_autonomous PRIVATE OSCAR_ENABLE_MOTOR_OUTPUT=1)` and rebuild/reflash. Do NOT enable on an unverified wired chassis.

From `firmware/` with Pico SDK configured: `cmake -S . -B build && cmake --build build --target oscar_autonomous -j2`. Firmware has **not** been cross-compiled in this artifact session: source must be compiled in the user's Pico SDK workspace, then tested with UART loopback/logic analyzer before motor power.

UART0 GP0 TX, GP1 RX, 115200 baud. Never drive GP5. Protocol is `HELLO OSCAR_V1` heartbeat, host commands `ARM`, `VEL vx vy wz`, `STOP`, `DISARM`; telemetry `ENC rl rr` every 100ms. Watchdog disarms 250ms after last `VEL` (a mere `STOP` does not keep motion armed). Host bridge additionally requires explicit /pico/arm true and sensor freshness; physical rollout requires hardwired emergency stop as well.

**The 0.20m rotation arm and motor direction signs are provisional.** Rear left invert and encoder signs are unconfirmed following Ethan's rewire. Calibration is essential.
