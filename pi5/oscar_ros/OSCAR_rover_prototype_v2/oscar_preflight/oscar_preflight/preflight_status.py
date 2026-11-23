"""Subscribe to existing topics and periodically report readiness without sending motion."""
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class Preflight(Node):
    def __init__(self):
        super().__init__('oscar_preflight_status')
        self.times = {}
        self.create_subscription(LaserScan, '/scan', lambda m: self.mark('scan'), 10)
        self.create_subscription(Odometry, '/odom', lambda m: self.mark('odom'), 10)
        self.create_subscription(String, '/outlets/detections', lambda m: self.mark('detections'), 10)
        self.create_timer(5.0, self.report)

    def mark(self, topic): self.times[topic] = time.monotonic()

    def report(self):
        age = time.monotonic()
        status = {name: ('recent' if age-self.times.get(name, -1e9) < 3 else 'missing/stale') for name in ('scan','odom','detections')}
        self.get_logger().info(f'Preflight topics: {status} | This does not certify hardware safety')


def main():
    rclpy.init()
    node = Preflight()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
