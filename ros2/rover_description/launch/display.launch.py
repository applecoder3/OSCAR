"""Preview geometry and wheel joints. No physics, driving or sensor data yet."""
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("rover_description"))
    description = xacro.process_file(
        str(share / "urdf" / "oscar.urdf.xacro"),
        mappings={"dimensions_file": str(share / "config" / "dimensions.yaml")},
    ).toxml()
    return LaunchDescription([
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{"robot_description": description, "use_sim_time": False}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
            parameters=[{"robot_description": description, "use_sim_time": False}],
            additional_env={"QT_QPA_PLATFORM": "xcb"},
        ),
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", str(share / "rviz" / "oscar.rviz")],
            parameters=[{"use_sim_time": False}],
            additional_env={"QT_QPA_PLATFORM": "xcb"}, output="screen",
        ),
    ])
