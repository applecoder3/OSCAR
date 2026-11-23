from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='oscar_preflight', executable='mock_odometry', output='screen',
             parameters=[{'allow_simulation': True, 'cmd_topic': '/mock_cmd_vel'}]),
        Node(package='oscar_preflight', executable='preflight_status', output='screen'),
    ])
