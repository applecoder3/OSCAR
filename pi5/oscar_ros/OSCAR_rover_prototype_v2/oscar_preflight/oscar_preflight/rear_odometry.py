"""Restricted vx/yaw odometry using RL/RR wheel ticks, vy assumed zero.

Two rear encoders DO NOT uniquely observe arbitrary mecanum strafe. This module
is for non-strafing rover prototype maneuvers. Positive wheels = robot forward.
"""
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from .kinematics import parse_encoder_line, meters_per_tick


class RearOdometry(Node):
    def __init__(self):
        super().__init__('oscar_rear_odometry')
        self.declare_parameter('track_width_m', 0.40)  # PROVISIONAL; measure effective width
        self.declare_parameter('counts_per_rev', 2464.0)
        self.declare_parameter('wheel_diameter_m', 0.080)
        self.declare_parameter('encoder_sign_rl', 1)
        self.declare_parameter('encoder_sign_rr', 1)
        self.declare_parameter('timeout_s', 0.5)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('input_topic', '/pico/telemetry_raw')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.x = self.y = self.yaw = 0.0
        self.prev = None
        self.prev_stamp = None
        self.last_update = None
        self.vx = self.wz = 0.0
        self.pub = self.create_publisher(Odometry, '/odom', 20)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(String, self.get_parameter('input_topic').value, self.on_line, 100)
        self.create_timer(0.05, self.tick)
        self.get_logger().warn('Restricted two-rear-encoder odometry. vy MUST remain zero. Effective track width uncalibrated.')

    def on_line(self, msg):
        result = parse_encoder_line(msg.data)
        if result is None:
            return
        now = time.monotonic()
        signs = (int(self.get_parameter('encoder_sign_rl').value), int(self.get_parameter('encoder_sign_rr').value))
        curr = (int(result[0]) * signs[0], int(result[1]) * signs[1])
        if self.prev is None:
            self.prev, self.prev_stamp, self.last_update = curr, now, now
            return
        dt = now - self.prev_stamp
        if not 0.005 <= dt <= 1.0:
            self.prev, self.prev_stamp = curr, now
            self.vx = self.wz = 0.0
            return
        dr_ticks, dl_ticks = curr[1] - self.prev[1], curr[0] - self.prev[0]
        self.prev, self.prev_stamp, self.last_update = curr, now, now
        mpt = meters_per_tick(float(self.get_parameter('counts_per_rev').value),
                              float(self.get_parameter('wheel_diameter_m').value))
        width = float(self.get_parameter('track_width_m').value)
        if width <= 0.05:
            self.get_logger().error('Invalid effective track width')
            return
        dl, dr = dl_ticks * mpt, dr_ticks * mpt
        ds, da = (dr + dl) * 0.5, (dr - dl) / width
        self.x += ds * math.cos(self.yaw + da * 0.5)
        self.y += ds * math.sin(self.yaw + da * 0.5)
        self.yaw = math.atan2(math.sin(self.yaw + da), math.cos(self.yaw + da))
        self.vx, self.wz = ds / dt, da / dt

    def tick(self):
        if self.last_update is None:
            return  # No made-up odometry before first encoder message
        if time.monotonic() - self.last_update > float(self.get_parameter('timeout_s').value):
            self.vx = self.wz = 0.0
            return  # Fail stale; do not continue publishing transform as live odom
        now = self.get_clock().now().to_msg()
        qz, qw = math.sin(self.yaw/2), math.cos(self.yaw/2)
        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = self.get_parameter('odom_frame').value
        msg.child_frame_id = self.get_parameter('base_frame').value
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = self.vx, self.wz
        # Planar pose covariance, inflated yaw due to mecanum slip / two encoders
        for idx, val in [(0, 0.025), (7, 0.025), (14, 999.0), (21, 999.0), (28, 999.0), (35, 0.15)]:
            msg.pose.covariance[idx] = val
        for idx, val in [(0, 0.05), (7, 999.0), (14, 999.0), (21, 999.0), (28, 999.0), (35, 0.2)]:
            msg.twist.covariance[idx] = val
        self.pub.publish(msg)
        if bool(self.get_parameter('publish_tf').value):
            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = msg.child_frame_id
            tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            self.tf.sendTransform(tf)


def main():
    rclpy.init()
    node = RearOdometry()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
