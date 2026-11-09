#!/usr/bin/env python3
"""Integration check: start an isolated server, inspect scans, test each drive axis.

Run from a sourced WSL ROS workspace; closes only the launch process group it creates.
Writes a result JSON and log into the caller-specified output directory.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ["ROS_DOMAIN_ID"] = "42"
os.environ["GZ_PARTITION"] = "oscar_sim"
os.environ["QT_QPA_PLATFORM"] = "xcb"

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from rclpy.qos import qos_profile_sensor_data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    report = {"status": "started"}
    log = (args.output / "gazebo.log").open("w")
    process = subprocess.Popen(
        ["ros2", "launch", "rover_sim", "sim.launch.py", "headless:=true", "rviz:=false"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
    )
    rclpy.init()
    node = rclpy.create_node("oscar_integration_check")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    messages = {}
    counts = {"scan": 0, "odom": 0, "truth": 0, "joints": 0}
    def receive(key, message):
        messages[key] = message
        counts[key] += 1
    for key, cls, topic in (("scan", LaserScan, "/scan"), ("odom", Odometry, "/odom"),
                            ("truth", Odometry, "/ground_truth/odom"),
                            ("joints", JointState, "/joint_states")):
        node.create_subscription(cls, topic, lambda msg, key=key: receive(key, msg), qos_profile_sensor_data)
    def spin(seconds, command=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if command is not None:
                publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.05)
    def pose(key):
        p = messages[key].pose.pose
        q = p.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return [p.position.x, p.position.y, yaw]
    try:
        deadline = time.monotonic() + 40
        while len(messages) < 4 and time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("Simulation exited before publishing all topics")
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(messages) != 4:
            raise RuntimeError(f"Missing messages; received {list(messages)}")
        spin(2, Twist())
        scan = messages["scan"]
        assert scan.header.frame_id == "lidar_link", scan.header.frame_id
        assert len(scan.ranges) == 720, len(scan.ranges)
        finite = [r for r in scan.ranges if math.isfinite(r)]
        assert finite and min(finite) > 0.2
        expected = {side+"_wheel_joint" for side in ("front_left", "front_right", "rear_left", "rear_right")}
        assert set(messages["joints"].name) == expected
        assert messages["odom"].header.frame_id == "odom"
        assert messages["odom"].child_frame_id == "base_footprint"
        report["scan"] = {"samples": len(scan.ranges), "frame": scan.header.frame_id,
                           "min_range_m": min(finite), "max_range_m": max(finite)}
        report["motion"] = []
        for label, velocity, duration in (("forward", (0.15, 0, 0), 2),
                                           ("left", (0, 0.15, 0), 2),
                                           ("turn_ccw", (0, 0, 0.4), 1.5)):
            before = pose("truth")
            command = Twist()
            command.linear.x, command.linear.y, command.angular.z = map(float, velocity)
            spin(duration, command)
            # No explicit stop command: verify the input timeout actually stops it.
            spin(1.3)
            after = pose("truth")
            delta = [after[i]-before[i] for i in range(3)]
            report["motion"].append({"command": label, "ground_truth_delta": delta,
                                     "odom": pose("odom")})
            print(label, delta, flush=True)
            if label == "forward":
                assert 0.1 < delta[0] < 0.7 and abs(delta[1]) < 0.07, delta
            elif label == "left":
                assert 0.1 < delta[1] < 0.7 and abs(delta[0]) < 0.07, delta
            else:
                assert 0.2 < delta[2] < 1.2 and math.hypot(*delta[:2]) < 0.07, delta
            spin(0.5)
            stopped = pose("truth")
            assert math.hypot(stopped[0]-after[0], stopped[1]-after[1]) < 0.015
            assert abs(stopped[2]-after[2]) < 0.03
        report["message_counts"] = counts
        report["status"] = "passed"
    except Exception as exc:
        report["status"] = "failed"
        report["error"] = repr(exc)
        raise
    finally:
        publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        # Let launch forward SIGINT once; signalling its whole group duplicates it.
        process.send_signal(signal.SIGINT) if process.poll() is None else None
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10)
        log.close()
        (args.output / "result.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
