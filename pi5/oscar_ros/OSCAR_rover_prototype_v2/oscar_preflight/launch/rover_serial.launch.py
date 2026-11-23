"""Serial disabled by default. Explicit enable_hardware and manual /pico/arm required."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_hardware', default_value='false'),
        DeclareLaunchArgument('port', default_value='/dev/serial0'),
        Node(package='oscar_preflight', executable='serial_bridge', output='screen',
             parameters=[{'enable_hardware': LaunchConfiguration('enable_hardware'),
                          'port': LaunchConfiguration('port')}]),
    ])
