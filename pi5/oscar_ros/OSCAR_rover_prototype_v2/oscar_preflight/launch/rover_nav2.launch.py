"""Nav2 navigation servers only; use SLAM separately for map->odom."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav_dir = get_package_share_directory('nav2_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value='/workspace/OSCAR_rover_prototype_v2/config/nav2_rover.yaml'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nav_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time':'false', 'autostart':'true',
                              'params_file':LaunchConfiguration('params_file'),
                              'use_composition':'false'}.items()),
    ])
