"""Start Gazebo, bridge, measured wheel states, command timeout and RViz."""
import importlib.util
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    share = Path(get_package_share_directory("rover_sim"))
    description = get_package_share_directory("rover_description")
    spec = importlib.util.spec_from_file_location("oscar_world_builder", share / "launch/world_builder.py")
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    # Retained for debugging; contains this launch's precise world/configuration.
    output = tempfile.mkdtemp(prefix="oscar_sim_")
    world, urdf = builder.build_world(description, output)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    gz_command = ["gz", "sim", "-r", "-v", "2"]
    if headless:
        gz_command += ["-s"]
    gz_command += [world]
    print(f"Oscar generated world: {world}")
    return [
        ExecuteProcess(cmd=gz_command, output="screen", additional_env={"QT_QPA_PLATFORM": "xcb"}),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             parameters=[{"config_file": str(share / "config/bridge.yaml"), "use_sim_time": True}], output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf, "use_sim_time": True}], output="screen"),
        Node(package="rover_sim", executable="command_guard.py", output="screen"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(share / "rviz/sim.rviz")],
             parameters=[{"use_sim_time": True}], additional_env={"QT_QPA_PLATFORM": "xcb"},
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        # Fixed isolation for this simulator. Other terminals must use domain 42.
        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),
        SetEnvironmentVariable("GZ_PARTITION", "oscar_sim"),
        OpaqueFunction(function=start),
    ])
