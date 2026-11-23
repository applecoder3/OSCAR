# OSCAR — Hardware Integration Handoff

## Software completed
- ROS 2 Jazzy and Nav2 installed in Docker
- LD19 LiDAR integrated with /scan
- SLAM Toolbox configured
- IMX708 camera capture running automatically
- YOLO ONNX outlet detector publishing /outlets/detections
- Outlet targeting connected to LiDAR and Nav2
- Rear encoder odometry implemented and software-tested
- Pico velocity firmware compiled
- Automatic startup configured with systemd

## Firmware
File: firmware/oscar_velocity.uf2

Firmware supports:
- ARM
- VEL vx vy wz
- STOP / DISARM
- HELLO OSCAR_V1 heartbeat
- ENC rearLeft rearRight telemetry
- 300 ms command watchdog

Motor output is compiled enabled, but firmware starts disarmed.

## Raspberry Pi connections
LD19 LiDAR: /dev/ttyUSB0
Pico UART: /dev/ttyAMA0
UART baud: 115200

Pi GPIO14 TX -> Pico GP1 RX
Pi GPIO15 RX <- Pico GP0 TX
Pi GND <-> Pico GND

Use 3.3 V UART logic. Verify wiring before applying power.

## Startup
Normal startup is automatic through:
- oscar.service
- oscar-camera.service

Current oscar.service runs software-test mode with MOCK odometry.

Hardware launcher:
scripts/start_oscar_hardware.sh

Hardware launcher defaults to enable_hardware=false.
Do not run both launchers simultaneously.

## Required physical validation
1. Mount and wire hardware.
2. Flash oscar_velocity.uf2 onto the Pico.
3. Verify UART heartbeat and encoder messages.
4. Verify motor directions with wheels raised.
5. Test STOP, DISARM and watchdog with motor power safely controlled.
6. Verify rear encoder signs and counts.
7. Measure wheelbase/track dimensions and calibrate odometry.
8. Calibrate camera/LiDAR alignment.
9. Verify SLAM, Nav2 and obstacle avoidance.
10. Test outlet detection and candidate goal generation.
11. Only then enable controlled autonomous driving.

## Important limitations
- Two rear encoders do not provide full mecanum strafing odometry.
- Navigation must remain restricted to non-strafing motion.
- Motor output has not been physically verified.
- Outlet detection accuracy has not been validated on mounted hardware.
- LiDAR-to-camera geometry is provisional.
- No unattended autonomous driving before physical safety testing.
