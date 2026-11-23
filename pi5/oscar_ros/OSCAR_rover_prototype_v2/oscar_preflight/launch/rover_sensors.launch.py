"""Real-sensor pipeline: encoders -> odom -> TF; no motors enabled."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('effective_track_width', default_value='0.40'),
        Node(package='oscar_preflight', executable='rear_odometry', output='screen',
             parameters=[{'track_width_m': LaunchConfiguration('effective_track_width')}]),
        Node(package='oscar_preflight', executable='preflight_status', output='screen'),
    ])
