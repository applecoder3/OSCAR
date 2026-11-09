#!/usr/bin/env python3
"""Hold/repeat a key to drive the simulator; release or Space stops it."""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "42":
        raise SystemExit("Simulation only: run export ROS_DOMAIN_ID=42 first.")
    if not sys.stdin.isatty():
        raise SystemExit("Run keyboard.py in an interactive Ubuntu terminal.")
    rclpy.init()
    node = rclpy.create_node("sim_keyboard")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    mapping = {"w": (0.15, 0, 0), "s": (-0.15, 0, 0),
               "a": (0, 0.15, 0), "d": (0, -0.15, 0),
               "q": (0, 0, 0.4), "e": (0, 0, -0.4)}
    previous = termios.tcgetattr(sys.stdin)
    print("SIMULATION: W/S forward/back; A/D strafe left/right; Q/E turn; Space stop; Ctrl+C exit.")
    print("Keep this terminal focused. Hold/repeat a key; release stops after 0.3 s.")
    last = 0.0
    command = Twist()
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1).lower()
                if key == "\x03":
                    break
                command = Twist()
                if key in mapping:
                    command.linear.x, command.linear.y, command.angular.z = map(float, mapping[key])
                last = time.monotonic()
            if time.monotonic() - last > 0.3:
                command = Twist()
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, previous)
        if rclpy.ok():
            for _ in range(5):
                publisher.publish(Twist())
                time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
