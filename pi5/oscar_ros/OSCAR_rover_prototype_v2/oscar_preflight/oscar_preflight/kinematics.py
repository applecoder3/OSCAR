"""Platform-independent mecanum kinematics and rear encoder parsing.

Ethan's hardware: rear encoders only, 2464 counts/rev, 80 mm wheel diameter.
Two rear encoders cannot recover all three planar velocities on a mecanum robot.
"""
import math
import re

COUNTS_PER_REV = 2464.0
WHEEL_DIAMETER_M = 0.080


def meters_per_tick(counts_per_rev=COUNTS_PER_REV, wheel_diameter_m=WHEEL_DIAMETER_M):
    if counts_per_rev <= 0 or wheel_diameter_m <= 0:
        raise ValueError('wheel parameters must be positive')
    return math.pi * wheel_diameter_m / counts_per_rev


def twist_to_wheels(vx, vy, wz, rotation_arm):
    """Return wheel linear speeds (FL, FR, RL, RR) in m/s, ROS body axes.

    Conventional X roller arrangement. Verify mecanum roller orientation and
    motor signs on blocks; NEVER assume that this is physically calibrated.
    """
    if rotation_arm <= 0:
        raise ValueError('rotation_arm must be positive')
    return (vx - vy - rotation_arm*wz,
            vx + vy + rotation_arm*wz,
            vx + vy - rotation_arm*wz,
            vx - vy + rotation_arm*wz)


def parse_encoder_line(line):
    """Proposed future line format: ENC <rl_count> <rr_count>.

    This is NOT currently emitted by Ethan's bench firmware.
    """
    match = re.fullmatch(r'ENC\s+([+-]?\d+)\s+([+-]?\d+)', line.strip())
    if not match:
        return None
    return int(match.group(1)), int(match.group(2))
