#!/usr/bin/env python3

"""
A variety of helper functions for general use.
"""

import numpy as np
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import constants as C

from typing import Tuple


def yaw_from_quaternion(quat: Quaternion) -> float:
    """
    Extract the yaw, in Euler angles, from a quaternion.
    """

    (_, _, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

    return yaw


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """
    Create a quaternion from the given yaw component in Euler angles.
    """
    components = quaternion_from_euler(ai=0.0, aj=0.0, ak=yaw)
    return Quaternion(*components)


def find_angle_offset(bot_pose: Pose, target_pose: Pose) -> float:
    """
    Find the relative angle from one pose to a target pose.
    """
    bot_yaw = yaw_from_quaternion(bot_pose.orientation)

    if bot_yaw < 0:
        # Convert angles to range [0, 2pi) for subtraction to work
        bot_yaw = bot_yaw + (2 * np.pi)

    target_angle = np.arctan2(
        target_pose.position.y - bot_pose.position.y,
        target_pose.position.x - bot_pose.position.x)

    if target_angle < 0:
        # Convert angles to range [0, 2pi) for subtraction to work
        target_angle = target_angle + (2 * np.pi)

    offset = target_angle - bot_yaw

    # Convert angles back to (-pi, pi] for proportional control to work
    if (offset > np.pi):
        offset = offset - (2 * np.pi)
    elif (offset < -np.pi):
        offset = offset + (2 * np.pi)

    return offset


def find_distance_offset(bot_pose: Pose, target_pose: Pose) -> Tuple[float, float]:
    """
    Find the distance offset in x and y coordinates from one pose to another.
    """
    return (
        target_pose.position.x - bot_pose.position.x,
        target_pose.position.y - bot_pose.position.y)


def find_distance(bot_pose: Pose, target_pose: Pose) -> float:
    """
    Find the distance between two poses.
    """
    return (
        (target_pose.position.x - bot_pose.position.x) ** 2 +
        (target_pose.position.y - bot_pose.position.y) ** 2) ** 0.5


def wrap_bounds(
        center_value: float,
        maximum_value: float,
        total_range: float
) -> tuple:
    """
    Create lower and upper bounds for a range that wraps around a given value.
    """
    lower_bound = (center_value - (total_range / 2)) % maximum_value
    upper_bound = (center_value + (total_range / 2)) % maximum_value
    return (lower_bound, upper_bound)


def compute_360_center(start: int, end: int) -> int:
    """
    Compute the center of two angles in degrees.
    """
    if start < end:
        return ((start + end + 360) // 2) % 360
    else:
        return (start + end) // 2


def get_closest_distance_and_angle(scan_data, front_angle_range):
    """
    Considering only angles that are in front of the robot (in C.FRONT_ANGLE_RANGE),
    return the shortest distance measurement and the corresponding angle.
    """
    scan_ranges = np.array(scan_data.ranges)
    # Set all things not in FRONT_ANGLE_RANGE = infinity
    scan_bounds = wrap_bounds(0, 360, front_angle_range)
    scan_ranges[np.arange(
        scan_bounds[1], scan_bounds[0]).astype(int)] = np.inf
    # Set all zero distance measurements to infinity (for a physical Turtlebot)
    scan_ranges[scan_ranges < C.ZERO_DISTANCE] = np.inf

    # Get closest angle
    distance_to_object = np.amin(scan_ranges)
    angle_to_object = np.argmin(scan_ranges)
    return (distance_to_object, angle_to_object)


def round_magnitude(number_to_round, magnitude):
    """
    Round a number up/down to a certain magnitude based on its sign.
    """
    abs_mag = abs(magnitude)
    if abs_mag < abs(number_to_round):
        return number_to_round
    else:
        return abs_mag * np.sign(number_to_round)
