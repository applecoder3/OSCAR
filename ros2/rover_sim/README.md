# Oscar simulation — driving and 2D LiDAR

## Start in Ubuntu 26.04 WSL

The package uses the dimensions in the installed `rover_description` package.
It regenerates a self-contained SDF world on every launch. No downloaded meshes
or models are required. Do not run the earlier display launch at the same time.

Terminal 1:

```bash
source /opt/ros/lyrical/setup.bash
source ~/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch rover_sim sim.launch.py
```

Gazebo and RViz open; the world runs immediately. Find the rover at the centre of
the room. The generated SDF path is printed and retained under `/tmp/oscar_sim_*`
for debugging. An empty RViz scan for a few seconds during startup is normal.

Terminal 2:

```bash
source /opt/ros/lyrical/setup.bash
source ~/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run rover_sim keyboard.py
```

Focus the terminal and hold/repeat keys:

| Key | Action |
| --- | --- |
| W / S | Forward / backward |
| A / D | Strafe left / right |
| Q / E | Rotate counterclockwise / clockwise |
| Space | Stop |
| Ctrl+C | Exit keyboard control |

Keyboard output stops after 0.3 seconds without a keystroke; a separate relay
zeros stale commands after 0.5 seconds and limits translation to 0.25 m/s and
rotation to 0.6 rad/s. Key-repeat delay can cause a brief initial pause. This is
manual driving: there is no obstacle-avoidance controller or Nav2 yet.
If the relay itself dies, pause/close Gazebo; the relay is not a hardware safety
system. Never connect this command topic to the real motors.

Terminal 3, inspect data:

```bash
source /opt/ros/lyrical/setup.bash
source ~/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic list
ros2 topic hz /scan
```

Stop `hz` with Ctrl+C before another command. Expected scan configuration is
10 scans per simulated second, 720 samples, frame `lidar_link`. Wall-clock topic
rate may be lower if the simulator runs below real time.

## What this stage contains

- Three-tier rover, four driven joints and Gazebo MecanumDrive controller.
- Roughly 6 m square room (5.9 m internal clearance), two tall obstacles and
  one red 0.12 m tall obstacle below the 0.35 m LiDAR plane.
- Generic 360-degree LiDAR: 0.15–12 m range, 10 Hz, 720 samples, 0.01 m Gaussian
  range-noise standard deviation. These are assumptions, NOT a verified D500 model.
- `/odom`, `/tf`, `/joint_states`, `/scan` and `/clock` bridged to ROS.
- Wheel odometry is the input for later SLAM; `/ground_truth/odom` is evaluation
  data only. Ground truth does not publish into the ROS TF tree.
- ROS domain 42 and Gazebo partition `oscar_sim` isolate this simulator. Physical
  testing should use a different domain (for example 43), with `use_sim_time=false`.
- No simulated camera images, YOLO, mapping, Nav2 or physical LiDAR driver yet.

## Physics limits

Wheel visuals remain cylinders. Sphere contacts with anisotropic friction use
Gazebo's mecanum approximation; individual rollers, motor torque response,
encoder quantisation and real floor slip are not modelled or calibrated.
Masses/inertias and geometry remain placeholders. The simulated wheel collision
envelope extends to +/- (track/2 + wheel_radius) = +/-0.20 m laterally, beyond
the visible wheel sides; use the conservative collision envelope when adding Nav2.
The physical rover has only two encoders; simulation observes all four joints.
Excellent simulated odometry agreement therefore does not establish real accuracy.

Reference: https://github.com/gazebosim/gz-sim/blob/gz-sim10/examples/worlds/mecanum_drive.sdf

## Acceptance checks and report evidence

1. Forward motion increases odom X; strafe left increases Y; CCW rotation increases yaw.
2. RViz shows the robot and scan without missing-transform errors.
3. Tall boxes and room walls appear in scans; the low red obstacle does not.
4. Releasing keys / stopping input brings the rover to rest.
5. Ground-truth pose and wheel odometry are recorded separately for comparisons.

Record a short run in a new Ubuntu terminal (domain 42):

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=42
mkdir -p ~/rover_test_data
cd ~/rover_test_data
ros2 bag record -o driving_trial_01 /clock /scan /odom /ground_truth/odom /tf /tf_static /cmd_vel /joint_states
```

Use a new output name for each trial. Stop with Ctrl+C. Record the dimensions,
world, software versions and purpose with the bag. A screenshot of scan points
is evidence of sensor integration, not yet a SLAM map or autonomous navigation.

## Validation

An integration test is provided as `check_sim.py` (source package, not a ROS node).
It starts its own headless server, checks message/frame content, exercises forward,
sideways and yaw commands, verifies the stale-input stop, then shuts down its launch.
Only run it when no other Oscar simulator is running in domain 42.

```bash
source /opt/ros/lyrical/setup.bash
source ~/rover_ws/install/setup.bash
python3 ~/rover_ws/src/rover_sim/check_sim.py /tmp/oscar-check
```

Headless runtime validation succeeded on the user's WSL/Gazebo 10.5.0 setup.
The initial run received 720-point scans and tested approximately 0.375 m forward,
0.375 m sideways, and 0.798 rad rotation, including the command-timeout interval.
These are integration-test observations, not commanded-distance accuracy trials.
User GUI verification of the new combined launch is still required.

If scans are missing, inspect the Gazebo log for rendering errors; GPU LiDAR
requires the renderer even in server-only mode. On the tested WSL system, EGL
device-permission warnings appeared but scans were still produced. Do not assume
warnings are harmless if `/scan` is absent.
