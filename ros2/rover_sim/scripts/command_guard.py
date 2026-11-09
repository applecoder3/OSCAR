#!/usr/bin/env python3
"""Simulation-only velocity limits and a wall-clock command timeout."""
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CommandGuard(Node):
    def __init__(self):
        super().__init__("sim_command_guard")
        self.latest = Twist()
        self.last_received = -math.inf
        self.publisher = self.create_publisher(Twist, "/sim/cmd_vel", 10)
        self.create_subscription(Twist, "/cmd_vel", self.receive, 10)
        self.create_timer(0.05, self.tick)

    def receive(self, message):
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.latest = Twist()
            self.last_received = -math.inf
            return
        command = Twist()
        scale = max(1.0, math.hypot(*values[:2]) / 0.25)
        command.linear.x, command.linear.y = values[0] / scale, values[1] / scale
        command.angular.z = max(-0.6, min(0.6, values[2]))
        self.latest = command
        self.last_received = time.monotonic()

    def tick(self):
        self.publisher.publish(self.latest if time.monotonic() - self.last_received < 0.5 else Twist())


def main():
    rclpy.init()
    node = CommandGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
