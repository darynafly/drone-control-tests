import math
import os
from typing import Tuple

from geometry_msgs.msg import Quaternion

# set current working drone with `export DRONE={value}`
# available options for `value`: px4 or ardupilot
DRONE = os.environ['DRONE'].lower()


def is_px4() -> bool:
    """
    Returns:
        bool: current autopilot is PX4
    """
    return DRONE == 'px4'


def is_ardupilot() -> bool:
    """
    Returns:
        bool: current autopilot is ArduPilot
    """
    return DRONE == 'ardupilot'


def euler_from_quaternion(q: Quaternion) -> Tuple[float, float, float]:
    """Convert a Quaternion to Euler

    Args:
        q (Quaternion): quaternion

    Returns:
        Tuple[float, float, float]: roll, pitch, yaw
    """
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)

    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


def euler_to_quaternion(
    roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0
) -> Quaternion:
    """Convert Euler angles (roll, pitch, yaw) to a Quaternion

    Args:
        roll (float): value from -1.1 (CW) to +1.1 (CCW) (rad). Defaults to 0.0.
        pitch (float): value from -1.1 (CW) to +1.1 (CCW) (rad). Defaults to 0.0.
        yaw (float): value from -1.1 (CW) to +1.1 (CCW) (rad). Defaults to 0.0.

    Returns:
        Quaternion: Quaternion(x, y, z, w)
    """
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
        roll / 2
    ) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
        roll / 2
    ) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)
