from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mock_odometry', default_value='false'),
        # Only run this launch in physical sensor-only mode by default.
        # Mock odom is not started automatically to prevent false localization.
        Node(package='oscar_preflight', executable='preflight_status', output='screen'),
        Node(package='oscar_preflight', executable='encoder_monitor', output='screen'),
        Node(package='oscar_preflight', executable='outlet_approach', output='screen',
             parameters=[{'enable_navigation_goals': False}]),
    ])
