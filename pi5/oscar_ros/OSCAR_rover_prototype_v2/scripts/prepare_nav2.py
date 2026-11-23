#!/usr/bin/env python3
"""Preserve Jazzy's installed complete Nav2 params; apply OSCAR frame/scan/limits."""
from pathlib import Path
import sys
import yaml

if len(sys.argv) != 3:
    raise SystemExit('Usage: prepare_nav2.py /opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml output.yaml')
src = Path(sys.argv[1]); dest = Path(sys.argv[2]); data = yaml.safe_load(src.read_text())
if not isinstance(data, dict): raise SystemExit('Not a Nav2 params map')
# Do not guess final robot shape; conservative provisional footprint, to be measured.
footprint = '[[0.16,0.13],[0.16,-0.13],[-0.16,-0.13],[-0.16,0.13]]'
for name in ('global_costmap', 'local_costmap'):
    if name not in data: raise SystemExit(f'Installed Jazzy default missing {name}; inspect template')
    section = data[name]['global_costmap' if name=='global_costmap' else 'local_costmap']['ros__parameters']
    section['robot_base_frame'] = 'base_link'
    section['global_frame'] = 'map' if name=='global_costmap' else 'odom'
    section['footprint'] = footprint
    section.pop('robot_radius', None)
    section['resolution'] = 0.05
    section['transform_tolerance'] = 0.3
    for plugin_name in ('obstacle_layer', 'voxel_layer'):
        plugin = section.get(plugin_name)
        if isinstance(plugin, dict):
            plugin['observation_sources'] = 'scan'
            plugin['scan'] = {'topic': '/scan', 'max_obstacle_height': 1.0,
                              'clearing': True, 'marking': True, 'data_type':'LaserScan',
                              'raytrace_max_range': 4.0, 'obstacle_max_range': 3.0}
    section['publish_frequency'] = 1.0
    section['update_frequency'] = 5.0
# Disable holonomic lateral movement and limit speeds for two-encoder nonstrafing mode.
controller = data.get('controller_server',{}).get('ros__parameters',{})
controller['controller_frequency'] = 10.0
controller['min_x_velocity_threshold'] = 0.01
controller['min_y_velocity_threshold'] = 0.01
controller['min_theta_velocity_threshold'] = 0.01
for label in controller.get('controller_plugins', ['FollowPath']):
    plugin = controller.get(label, {})
    if 'dwb_core::DWBLocalPlanner' in plugin.get('plugin',''):
        plugin.update({'min_vel_x':-0.05, 'max_vel_x':0.15,'min_vel_y':0.0, 'max_vel_y':0.0,
                       'max_vel_theta':0.4, 'min_speed_xy':0.0, 'max_speed_xy':0.15,
                       'min_speed_theta':0.0,'acc_lim_x':0.15,'decel_lim_x':-0.15,
                       'acc_lim_y':0.0, 'decel_lim_y':0.0,'acc_lim_theta':0.35,
                       'decel_lim_theta':-0.35, 'vx_samples':10,'vy_samples':1,'vtheta_samples':10})
    if 'RegulatedPurePursuitController' in plugin.get('plugin',''):
        plugin.update({'desired_linear_vel':0.12,'max_angular_accel':0.35,
                       'allow_reversing':False})
for name in ('bt_navigator','behavior_server','waypoint_follower'):
    params = data.get(name, {}).get('ros__parameters',{})
    if 'robot_base_frame' in params: params['robot_base_frame']='base_link'
    if 'global_frame' in params: params['global_frame']='map'
# Velocity smoother output is typically /cmd_vel_smoothed; physical bridge consumes /cmd_vel.
# Do not silently remap. Use a single confirmed command topic after Nav2 smoke test.
dest.parent.mkdir(parents=True,exist_ok=True)
dest.write_text('# Generated from installed Jazzy Nav2 defaults; provisional footprint and effective track width.\n'+yaml.safe_dump(data,sort_keys=False))
print('WROTE',dest)
