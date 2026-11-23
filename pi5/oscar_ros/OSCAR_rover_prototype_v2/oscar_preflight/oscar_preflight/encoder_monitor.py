"""Parse proposed Pico rear encoder telemetry; never fabricates full mecanum odom."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from .kinematics import parse_encoder_line, COUNTS_PER_REV
import math


class EncoderMonitor(Node):
    def __init__(self):
        super().__init__('oscar_encoder_monitor')
        self.declare_parameter('counts_per_rev', COUNTS_PER_REV)
        self.declare_parameter('encoder_sign_rl', 1)
        self.declare_parameter('encoder_sign_rr', 1)
        self.declare_parameter('input_topic', '/pico/telemetry_raw')
        self.pub = self.create_publisher(JointState, '/pico/rear_wheels', 10)
        self.sub = self.create_subscription(String, self.get_parameter('input_topic').value, self.on_line, 10)
        self.get_logger().warn('ENC input is PROPOSED, not emitted by Ethan bench firmware. No /odom from 2 rear encoders.')

    def on_line(self, msg):
        parsed = parse_encoder_line(msg.data)
        if parsed is None: return
        counts = [parsed[0] * int(self.get_parameter('encoder_sign_rl').value),
                  parsed[1] * int(self.get_parameter('encoder_sign_rr').value)]
        q = JointState()
        q.header.stamp = self.get_clock().now().to_msg()
        q.name = ['rear_left_wheel_joint', 'rear_right_wheel_joint']
        ticks = float(self.get_parameter('counts_per_rev').value)
        if ticks <= 0: return
        q.position = [v*2*math.pi/ticks for v in counts]
        self.pub.publish(q)


def main():
    rclpy.init()
    node = EncoderMonitor()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
