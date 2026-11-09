# Oscar robot description — geometry preview

This package implements step 2: a parameterized robot description with three
plates, four wheel joints, posts, a Pi placeholder, LiDAR frame and camera frame.
The near end in the supplied chassis photo is the front (+X); +Y is left and +Z up.

## Scope and limitations

- All dimensions, masses, effort/velocity limits and inertias are provisional.
- Wheel cylinders are placeholders for layout. They do not simulate mecanum
  rollers, slip or lateral contact behavior. Driving physics belongs in the next step.
- Camera and LiDAR are bodies/frames only. There is no image or LaserScan output.
- Joint-state sliders demonstrate wheel rotation; they do not move the chassis.
- This preview uses wall-clock time, not Gazebo simulation time. Close it before
  starting the later Gazebo launch to avoid duplicate state/TF publishers.
- Battery, motor geometry, wiring and detailed mounting hardware are omitted;
  the current mass distribution is not suitable for stability/performance claims.
- The package.xml maintainer and license fields are placeholders for the team.

## Install and run (Ubuntu 26.04 WSL, ROS Lyrical)

Install dependencies:

```bash
sudo apt update
sudo apt install ros-lyrical-xacro ros-lyrical-robot-state-publisher ros-lyrical-joint-state-publisher-gui ros-lyrical-rviz2
```

Copy this package to `~/rover_ws/src/rover_description` (save any existing edits first).
Then:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/rover_ws
colcon build --symlink-install --packages-select rover_description
source install/setup.bash
ros2 launch rover_description display.launch.py
```

The launch file applies `QT_QPA_PLATFORM=xcb` to both GUI applications for WSLg.
Expected result: three plates, four wheels touching the ground grid, green Pi on
the middle plate, red LiDAR on the top pedestal, blue camera near the front,
and a red front marker. The four joint sliders rotate the silver wheel spokes.

## Change dimensions

Edit `config/dimensions.yaml` inside your WSL source package. Units are metres,
kilograms and radians. `wheelbase` and `track` are distances between wheel centres,
not overall body dimensions. Heights refer to centres above the ground.
Positive camera tilt points its +X viewing direction downward.

Preserve physically sensible clearances when editing: the LiDAR pedestal height
must stay positive, plates must remain ordered, and wheels must not intersect plates.
Rebuild and relaunch after changes. Do not edit generated files in `build/`.

## Geometry acceptance checks

1. RViz Fixed Frame is `base_footprint`; RobotModel has no missing-transform error.
2. Four wheel sliders correspond to the correct front/rear and left/right names.
3. Wheels touch Z=0 and are outside the plates.
4. Red X axis points forward; camera optical +Z points forward and down.
5. LiDAR scan frame is clear of the top plate and camera.

Keep screenshots and the dimensions file with your report, explicitly labelled
"provisional geometry preview". No mapping/navigation or physical accuracy has
been established at this stage.
