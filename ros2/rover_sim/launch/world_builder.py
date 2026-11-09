"""Generate a self-contained room from the installed, editable URDF dimensions.

Mecanum sphere contacts with body-relative anisotropic friction follow Gazebo's
documented example, not a calibrated roller/slip model. No external meshes needed.
"""
import math
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import xacro
import yaml

GZ_NS = "http://gazebosim.org/schema"
ET.register_namespace("gz", GZ_NS)


def element(parent, tag, text=None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if text is not None:
        child.text = str(text)
    return child


def plugin(parent, filename, name, **parameters):
    node = element(parent, "plugin", filename=filename, name=name)
    for key, value in parameters.items():
        element(node, key, value)
    return node


def static_box(world, name, pose, size, color):
    model = element(world, "model", name=name)
    element(model, "static", "true")
    element(model, "pose", pose)
    link = element(model, "link", name="body")
    for tag in ("collision", "visual"):
        body = element(link, tag, name=tag)
        element(element(element(body, "geometry"), "box"), "size", size)
        if tag == "visual":
            material = element(body, "material")
            element(material, "ambient", color)
            element(material, "diffuse", color)


def build_world(description_share, output_directory):
    description_share = Path(description_share)
    output_directory = Path(output_directory)
    config = description_share / "config/dimensions.yaml"
    d = yaml.safe_load(config.read_text())
    if not (0 < d["bottom_height"] < d["middle_height"] < d["top_height"]):
        raise ValueError("Plate heights must be positive and ordered bottom < middle < top")
    doc = xacro.process_file(str(description_share / "urdf/oscar.urdf.xacro"),
                             mappings={"dimensions_file": str(config)})
    urdf_text = doc.toxml()
    urdf = ET.fromstring(urdf_text)
    # Keep an actual LiDAR link for the sensor; other fixed links may be lumped.
    gazebo = element(urdf, "gazebo", reference="lidar_mount")
    element(gazebo, "preserveFixedJoint", "true")
    source = output_directory / "oscar.urdf"
    ET.ElementTree(urdf).write(source, encoding="unicode")
    converted = subprocess.run(["gz", "sdf", "-p", str(source)],
                               check=True, capture_output=True, text=True)
    sdf_robot = ET.fromstring(converted.stdout)
    robot = sdf_robot.find("model")
    if robot is None or robot.find("link[@name='lidar_link']") is None:
        raise RuntimeError("URDF conversion did not retain the LiDAR link")
    robot.set("name", "oscar")
    # Spawn slightly above the floor and allow the contact solver to settle.
    element(robot, "pose", "0 0 0.01 0 0 0")
    body = robot.find("link").get("name")
    for name, friction_y in (("front_left", -1), ("front_right", 1),
                              ("rear_left", 1), ("rear_right", -1)):
        wheel = robot.find(f"link[@name='{name}_wheel']")
        if wheel is None:
            raise RuntimeError(f"Missing wheel link: {name}")
        for collision in wheel.findall("collision"):
            geometry = collision.find("geometry")
            geometry.clear()
            element(element(geometry, "sphere"), "radius", d["wheel_radius"])
            old_surface = collision.find("surface")
            if old_surface is not None:
                collision.remove(old_surface)
            friction = element(element(element(collision, "surface"), "friction"), "ode")
            element(friction, "mu", "1.0")
            element(friction, "mu2", "0.0")
            direction = element(friction, "fdir1", f"1 {friction_y} 0")
            direction.set(f"{{{GZ_NS}}}expressed_in", body)

    plugin(robot, "gz-sim-mecanum-drive-system", "gz::sim::systems::MecanumDrive",
           front_left_joint="front_left_wheel_joint", front_right_joint="front_right_wheel_joint",
           back_left_joint="rear_left_wheel_joint", back_right_joint="rear_right_wheel_joint",
           wheel_separation=d["track"], wheelbase=d["wheelbase"], wheel_radius=d["wheel_radius"],
           topic="/model/oscar/cmd_vel", odom_topic="/model/oscar/odometry",
           tf_topic="/model/oscar/tf", frame_id="odom", child_frame_id="base_footprint",
           odom_publish_frequency=30,
           min_linear_velocity=-0.25, max_linear_velocity=0.25,
           min_angular_velocity=-0.6, max_angular_velocity=0.6,
           min_linear_acceleration=-0.5, max_linear_acceleration=0.5,
           min_angular_acceleration=-1.0, max_angular_acceleration=1.0)
    joints = plugin(robot, "gz-sim-joint-state-publisher-system", "gz::sim::systems::JointStatePublisher",
                    topic="/model/oscar/joint_states")
    for name in ("front_left", "front_right", "rear_left", "rear_right"):
        element(joints, "joint_name", name + "_wheel_joint")
    # Evaluation only: never used as the navigation odometry or broadcast into TF.
    plugin(robot, "gz-sim-odometry-publisher-system", "gz::sim::systems::OdometryPublisher",
           odom_topic="/model/oscar/ground_truth", odom_frame="world",
           robot_base_frame="base_footprint", dimensions=3, odom_publish_frequency=30,
           tf_topic="/model/oscar/ground_truth_tf")

    lidar = robot.find("link[@name='lidar_link']")
    sensor = element(lidar, "sensor", name="lidar", type="gpu_lidar")
    element(sensor, "topic", "/model/oscar/scan")
    element(sensor, "frame_id", "lidar_link")
    element(sensor, "update_rate", 10)
    element(sensor, "always_on", "true")
    element(sensor, "visualize", "false")
    ray = element(sensor, "lidar")
    horizontal = element(element(ray, "scan"), "horizontal")
    for name, value in (("samples", 720), ("resolution", 1),
                         ("min_angle", -math.pi), ("max_angle", math.pi)):
        element(horizontal, name, value)
    ranges = element(ray, "range")
    for name, value in (("min", 0.15), ("max", 12.0), ("resolution", 0.01)):
        element(ranges, name, value)
    noise = element(ray, "noise")
    element(noise, "type", "gaussian")
    element(noise, "mean", 0)
    element(noise, "stddev", 0.01)

    sdf = ET.Element("sdf", {"version": "1.10"})
    world = element(sdf, "world", name="outlet_room")
    physics = element(world, "physics", name="default", type="ignored")
    element(physics, "max_step_size", 0.001)
    element(physics, "real_time_factor", 1.0)
    element(world, "gravity", "0 0 -9.81")
    plugin(world, "gz-sim-physics-system", "gz::sim::systems::Physics")
    plugin(world, "gz-sim-user-commands-system", "gz::sim::systems::UserCommands")
    plugin(world, "gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster")
    plugin(world, "gz-sim-sensors-system", "gz::sim::systems::Sensors", render_engine="ogre2")
    scene = element(world, "scene")
    element(scene, "ambient", "0.6 0.6 0.6 1")
    element(scene, "background", "0.75 0.8 0.85 1")
    light = element(world, "light", name="sun", type="directional")
    element(light, "pose", "0 0 8 0 0 0")
    element(light, "diffuse", "0.8 0.8 0.8 1")
    element(light, "direction", "-0.4 -0.3 -1")
    element(light, "cast_shadows", "true")
    static_box(world, "floor", "0 0 -0.05 0 0 0", "7 7 0.1", "0.7 0.7 0.7 1")
    for name, pose, size in (
        ("north_wall", "0 3 0.5 0 0 0", "6.1 0.1 1"),
        ("south_wall", "0 -3 0.5 0 0 0", "6.1 0.1 1"),
        ("east_wall", "3 0 0.5 0 0 0", "0.1 6.1 1"),
        ("west_wall", "-3 0 0.5 0 0 0", "0.1 6.1 1"),
    ):
        static_box(world, name, pose, size, "0.85 0.85 0.9 1")
    static_box(world, "box_a", "1.5 0 0.3 0 0 0", "0.6 0.6 0.6", "0.8 0.4 0.1 1")
    static_box(world, "box_b", "-1.2 1.3 0.35 0 0 0", "0.5 0.8 0.7", "0.2 0.5 0.8 1")
    static_box(world, "low_obstacle", "0.8 -1.0 0.06 0 0 0", "0.4 0.4 0.12", "0.9 0.15 0.15 1")
    world.append(robot)
    output = output_directory / "outlet_room.sdf"
    ET.indent(sdf)
    ET.ElementTree(sdf).write(output, encoding="unicode", xml_declaration=True)
    return str(output), urdf_text


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("description_share")
    parser.add_argument("output_directory")
    args = parser.parse_args()
    Path(args.output_directory).mkdir(parents=True, exist_ok=True)
    print(build_world(args.description_share, args.output_directory)[0])
