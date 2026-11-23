"""Pure geometric utilities. LiDAR bearing and camera calibration are not interchangeable."""
import math


def camera_bearing(center_x_norm, fov_rad):
    if not 0 <= center_x_norm <= 1 or not 0 < fov_rad < math.pi:
        raise ValueError('invalid normalized position/FOV')
    return math.atan((0.5-center_x_norm)*2.0*math.tan(fov_rad/2))


def laser_range_at_bearing(ranges, angle_min, angle_increment, bearing, window=3, range_min=0.05, range_max=10):
    if not ranges or angle_increment == 0: return None
    i = round((bearing-angle_min)/angle_increment)
    vals=[]
    for j in range(max(0,i-window), min(len(ranges), i+window+1)):
        v=ranges[j]
        if math.isfinite(v) and range_min <= v <= range_max: vals.append(v)
    return sorted(vals)[len(vals)//2] if vals else None


def approach_point(base_x, base_y, base_yaw, bearing, range_m, stop_distance):
    if range_m <= stop_distance or stop_distance <= 0: return None
    travel = range_m-stop_distance
    heading = base_yaw+bearing
    return (base_x+travel*math.cos(heading), base_y+travel*math.sin(heading), heading)
