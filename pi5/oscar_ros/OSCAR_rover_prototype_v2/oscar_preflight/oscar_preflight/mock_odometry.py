"""DEVELOPMENT-ONLY commanded-twist integrator, not physical odometry."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class MockOdom(Node):
    def __init__(self):
        super().__init__('oscar_mock_odometry')
        self.declare_parameter('allow_simulation', False)
        if not self.get_parameter('allow_simulation').value:
            raise RuntimeError('Mock odometry is DISABLED by default; set allow_simulation:=true for simulation only')
        self.declare_parameter('cmd_topic', '/mock_cmd_vel')
        self.declare_parameter('timeout_seconds', 0.3)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.wz = 0.0
        self.last_msg = self.get_clock().now()
        self.last_update = self.last_msg
        self.sub = self.create_subscription(Twist, self.get_parameter('cmd_topic').value, self.on_cmd, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().warn('SIMULATION ONLY: /odom integrates commands, NOT wheel measurements')

    def on_cmd(self, msg):
        self.vx, self.vy, self.wz = msg.linear.x, msg.linear.y, msg.angular.z
        self.last_msg = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        dt = max(0.0, min(0.2, (now - self.last_update).nanoseconds / 1e9))
        self.last_update = now
        if (now - self.last_msg).nanoseconds / 1e9 > float(self.get_parameter('timeout_seconds').value):
            self.vx = self.vy = self.wz = 0.0
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (self.vx*c - self.vy*s)*dt
        self.y += (self.vx*s + self.vy*c)*dt
        self.yaw = math.atan2(math.sin(self.yaw + self.wz*dt), math.cos(self.yaw + self.wz*dt))
        z, qw = quaternion(self.yaw)
        stamp = now.to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.get_parameter('odom_frame').value
        odom.child_frame_id = self.get_parameter('base_frame').value
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = z, qw
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = self.vx, self.vy
        odom.twist.twist.angular.z = self.wz
        # Deliberately large covariance: commanded velocity is NOT measurement.
        odom.pose.covariance[0] = odom.pose.covariance[7] = 1.0
        odom.pose.covariance[35] = 2.0
        self.pub.publish(odom)
        if self.get_parameter('publish_tf').value:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id, tf.child_frame_id = odom.header.frame_id, odom.child_frame_id
            tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
            tf.transform.rotation.z, tf.transform.rotation.w = z, qw
            self.tf.sendTransform(tf)


def main():
    rclpy.init()
    node = None
    try:
        node = MockOdom()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as exc:
        print(exc)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
