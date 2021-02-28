#!/usr/bin/env python3
import numpy as np
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def yaw_from_quaternion(q: Quaternion) -> float:
    """
    Extract the yaw, in Euler angles, from a quaternion.
    """

    (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    return yaw


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """
    Create a quaternion from the given yaw component in Euler angles.
    """
    components = quaternion_from_euler(ai=0.0, aj=0.0, ak=yaw)
    return Quaternion(*components)


def find_angle_offset(bot_pose: Pose, target_pose: Pose) -> float:
    bot_yaw = yaw_from_quaternion(bot_pose.orientation)
    target_angle = np.arctan2(
        target_pose.position.y - bot_pose.position.y,
        target_pose.position.x - bot_pose.position.x)
    return target_angle - bot_yaw


def find_distance_offset(bot_pose: Pose, target_pose: Pose) -> float:
    return (
        target_pose.position.x - bot_pose.position.x,
        target_pose.position.y - bot_pose.position.y)


def find_distance(bot_pose: Pose, target_pose: Pose) -> float:
    return (
        (target_pose.position.x - bot_pose.position.x) ** 2 +
        (target_pose.position.y - bot_pose.position.y) ** 2) ** 0.5


def wrap_bounds(
        center_value: float,
        maximum_value: float,
        total_range: float
        ) -> tuple:
    lower_bound = (center_value - (total_range / 2)) % maximum_value
    upper_bound = (center_value + (total_range / 2)) % maximum_value
    return (lower_bound, upper_bound)