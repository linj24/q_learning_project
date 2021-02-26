#!/usr/bin/env python3
import numpy as np
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def yaw_from_quaternion(q):
    """
    Extract the yaw, in Euler angles, from a quaternion.
    """

    (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    return yaw


def yaw_to_quaternion(yaw):
    """
    Create a quaternion from the given yaw component in Euler angles.
    """
    components = quaternion_from_euler(ai=0.0, aj=0.0, ak=yaw)
    return Quaternion(*components)


def find_angle_offset(bot_pose, target_pose):
    bot_yaw = yaw_from_quaternion(bot_pose.orientation)
    target_angle = np.arctan2(
        target_pose.position.y - bot_pose.position.y,
        target_pose.position.x - bot_pose.position.x)
    return target_angle - bot_yaw


def find_distance_offset(bot_pose, target_pose):
    return (
        target_pose.position.x - bot_pose.position.x,
        target_pose.position.y - bot_pose.position.y)


class ImgCentroidMsg(object):
    def __init__(self, color=None, centroid=(None, None)):
        self.color = color
        self.centroid = centroid


