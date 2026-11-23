"""Safety-gated candidate goal planner; never sends a goal unless manually enabled.

Camera and LiDAR bearing approximation is a provisional 2D geometry model:
requires calibrated camera horizontal FOV and camera-to-laser yaw. LiDAR hit
may be wall, not outlet. Do not use around people or with uncalibrated sensors.
"""
import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from .target_math import camera_bearing, laser_range_at_bearing, approach_point


class OutletApproach(Node):
    def __init__(self):
        super().__init__('oscar_outlet_approach')
        self.declare_parameter('enable_navigation_goals', False)
        self.declare_parameter('camera_horizontal_fov_deg', 62.0)
        self.declare_parameter('camera_to_laser_yaw_deg', 0.0)
        self.declare_parameter('target_confidence', 0.7)
        self.declare_parameter('maximum_target_range_m', 0.6)
        self.declare_parameter('standoff_m', 0.40)
        self.declare_parameter('scan_max_age_s', 0.4)
        self.declare_parameter('cooldown_seconds', 10.0)
        self.scan = None
        self.scan_time = None
        self.last_goal_time = None
        self.approved = False
        self.pub = self.create_publisher(PoseStamped, '/outlets/candidate_goal', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(String, '/outlets/detections', self.on_detections, 10)
        self.create_subscription(Bool, '/outlets/approve_once', self.on_approve, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().warn('Navigation DISABLED by default. Candidates only. Explicit one-time approval required.')

    def on_scan(self, msg):
        self.scan=msg
        self.scan_time=self.get_clock().now()

    def on_approve(self, msg):
        if msg.data:
            self.approved=True
            self.get_logger().warn('One-shot approval armed; must also have enable_navigation_goals=true')

    def on_detections(self, msg):
        try: data = json.loads(msg.data)
        except (ValueError, TypeError): return
        if self.scan is None or self.scan_time is None: return
        now=self.get_clock().now()
        if (now-self.scan_time).nanoseconds/1e9 > float(self.get_parameter('scan_max_age_s').value): return
        if self.last_goal_time is not None and (now-self.last_goal_time).nanoseconds/1e9 < float(self.get_parameter('cooldown_seconds').value): return
        choices=[d for d in data.get('detections',[]) if d.get('confidence',0) >= float(self.get_parameter('target_confidence').value)]
        if not choices: return
        best=max(choices, key=lambda d:d['confidence'])
        try:
            cx=float(best['center_x_norm'])
            bearing=camera_bearing(cx, math.radians(float(self.get_parameter('camera_horizontal_fov_deg').value)))
            # Camera yaw must be measured relative to LiDAR; current 0 is placeholder.
            laser_bearing=bearing+math.radians(float(self.get_parameter('camera_to_laser_yaw_deg').value))
            distance=laser_range_at_bearing(self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                laser_bearing, window=3, range_min=self.scan.range_min,
                range_max=min(float(self.get_parameter('maximum_target_range_m').value),self.scan.range_max))
            if distance is None: return
            tf=self.tf_buffer.lookup_transform('map','base_link',rclpy.time.Time())
        except Exception as exc:
            self.get_logger().debug(f'Candidate unavailable (TF/calibration): {exc}')
            return
        q=tf.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        candidate=approach_point(tf.transform.translation.x, tf.transform.translation.y,
                                 yaw, laser_bearing, distance, float(self.get_parameter('standoff_m').value))
        if candidate is None: return
        x,y,heading=candidate
        goal=PoseStamped()
        goal.header.frame_id='map'
        goal.header.stamp=now.to_msg()
        goal.pose.position.x=x; goal.pose.position.y=y
        goal.pose.orientation.z=math.sin(heading/2)
        goal.pose.orientation.w=math.cos(heading/2)
        self.pub.publish(goal)
        if not self.get_parameter('enable_navigation_goals').value: return
        if not self.approved: return
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('Nav2 action server unavailable; not sending goal')
            return
        # Consume approval before performing the send; no repeated commands.
        self.approved=False
        self.last_goal_time=now
        action=NavigateToPose.Goal()
        action.pose=goal
        self.nav_client.send_goal_async(action)
        self.get_logger().warn('Sent manually approved candidate goal; ensure Nav2 safety layers are configured')


def main():
    rclpy.init()
    node=OutletApproach()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
