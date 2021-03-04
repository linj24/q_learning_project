#!/usr/bin/env python3

"""
A variety of helper functions for general use.
"""

import numpy as np
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import constants as C


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
    target_angle = np.arctan2(
        target_pose.position.y - bot_pose.position.y,
        target_pose.position.x - bot_pose.position.x)
    return target_angle - bot_yaw


def find_distance_offset(bot_pose: Pose, target_pose: Pose) -> float:
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


def find_objects_in_scan(scan_data: LaserScan) -> list:
    """
    Find the relative angles of distinct objects the bot can detect with a laser scan.
    """
    scan_ranges = scan_data.ranges
    range_starts = []
    for angle, dist in enumerate(scan_ranges):
        if (scan_ranges[angle - 1] - dist) > C.OBJ_DIST_DIFF:
            range_starts.append(angle)

    # A range consists of a starting angle and an end angle.
    ranges = []
    for i, range_start in enumerate(range_starts):
        if not np.isinf(scan_ranges[range_starts[i - 1]]):
            obj_range = (range_starts[i - 1], range_start)
            ranges.append(obj_range)
    return ranges


def compute_360_center(start_end: tuple) -> int:
    """
    Compute the center of two angles in degrees.
    """
    start = start_end[0]
    end = start_end[1]
    if start < end:
        return ((start + end + 360) // 2) % 360
    else:
        return (start + end) // 2


def get_closest_distance_and_angle(scan_data):
    """
    Considering only angles that are in front of the robot (in C.FRONT_ANGLE_RANGE),
    return the shortest distance measurement and the corresponding angle.
    """
    scan_ranges = np.array(scan_data.ranges)
    # Set all things not in FRONT_ANGLE_RANGE = infinity
    scan_bounds = wrap_bounds(0, 360, C.FRONT_ANGLE_RANGE)
    scan_ranges[np.arange(
        scan_bounds[1], scan_bounds[0]).astype(int)] = np.inf

    # get closest angle
    distance_to_object = np.amin(scan_ranges)
    angle_to_object = np.argmin(scan_ranges)
    return (distance_to_object, angle_to_object)


def is_centered(scan_data):
    """
    Considering only angles that are in front of the robot (in C.FRONT_ANGLE_RANGE)
    returns True if the closest object is almost directly in front of the object
    (within C.CENTER_ANGLE_RANGE of straight ahead of the robot). False otherwise.
    """
    _, angle_to_object = get_closest_distance_and_angle(scan_data)

    return (angle_to_object < C.CENTER_ANGLE_RANGE
            or angle_to_object > 360 - C.CENTER_ANGLE_RANGE)


def round_magnitude(number_to_round, magnitude):
    """
    Round a number up/down to a certain magnitude based on its sign.
    """
    abs_mag = abs(magnitude)
    if abs_mag < abs(number_to_round):
        return number_to_round
    else:
        return abs_mag * np.sign(number_to_round)
