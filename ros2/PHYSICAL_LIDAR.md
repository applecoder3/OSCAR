# Physical D500: first bench check on the Pi

Keep the sensor stationary for the first check. Connect it through the supplied
kit USB adapter and a data-capable USB cable. Do not wire its bare connector to
the Pi GPIO as part of this check. Keep the spinning head clear of loose cables.

Run over SSH on the Pi (not in WSL):

```bash
lsusb
ls -l /dev/serial/by-id/
id -nG
```

Save the output. The serial adapter appearing establishes USB enumeration, not
valid distance data. The next step is selecting the actual serial device and
checking its data stream; then validating stationary ranges against measured
wall distances. A missing by-id directory can mean the serial device has not
enumerated, so inspect cable/adapter and kernel logs before installing software.

ROS is not currently installed on the Pi's Debian 13 system. Preserve its working
camera setup. For a real `/scan` topic, choose either a compatible container on
the Pi or another verified driver deployment route after the serial check.
Do not install Ubuntu ROS apt packages directly on Debian.

LDROBOT's `ldlidar_stl_ros2` driver supports LD19 and its LD19 launch uses 230400
baud. Verify the sensor label/model before choosing that configuration. The
default upstream launch assumes ttyUSB0 and a 0.18 m mounting height; those are
examples, not the measured frame transform or persistent device path for this rover.

https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2

Use ROS_DOMAIN_ID=43 for hardware, separate from simulation domain 42. Physical
nodes use wall-clock time (`use_sim_time=false`). Do not combine the real and
simulated `/scan`, `/odom`, or motor command streams.

## Measurements to gather

- Exact sensor label, adapter identity and persistent serial path.
- Packet/scan rate and timeouts while stationary.
- Mean/range variation against a wall at measured distances.
- Direction of increasing scan angle and forward-zero alignment.
- Actual sensor height and offset from the rover centre.
- Blind spots due to posts, camera, cables, low objects and reflective surfaces.

Keep motor power/control outside this sensor-only bench test. There is no
physical motor interface in the simulation package.
