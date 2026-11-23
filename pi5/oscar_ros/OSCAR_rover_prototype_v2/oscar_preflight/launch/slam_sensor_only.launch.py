"""SLAM Toolbox wiring. Requires REAL /odom + odom->base_link TF to map correctly.
The sensor can remain on the desk for a launch check, not a useful moving map.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('slam_params', default_value=join(get_package_share_directory('oscar_preflight'), 'config', 'slam.yaml')),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
             parameters=[LaunchConfiguration('slam_params'), {'use_sim_time': False}]),
    ])
