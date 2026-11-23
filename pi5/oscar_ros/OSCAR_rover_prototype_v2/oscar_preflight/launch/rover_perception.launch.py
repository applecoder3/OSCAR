"""Perception must be launched only when camera and inference runtime are verified."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='/workspace/OSCAR_rover_prototype_v2/models/exp.pt'),
        DeclareLaunchArgument('video_source', default_value='0'),
        Node(package='oscar_preflight', executable='outlet_detector', output='screen',
             parameters=[{'allow_camera': True,'model_path':LaunchConfiguration('model'),
                          'video_source': LaunchConfiguration('video_source')}]),
        Node(package='oscar_preflight', executable='outlet_approach', output='screen',
             parameters=[{'enable_navigation_goals': False}]),
    ])
