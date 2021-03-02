#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import constants as C
from utils import find_angle_offset, find_distance, wrap_bounds
from q_learning_project.msg import ActionState, ImgCen


def calculate_velocity_odom(odom_data: Odometry, target_pose: Pose) -> Twist:
    bot_vel = Twist()

    angle_offset = find_angle_offset(odom_data.pose.pose, target_pose)
    bot_vel.angular.z = C.KP_ANG * -angle_offset * C.CENTER_NAV_MODIFIER

    if abs(angle_offset) < (np.pi / 4):
        distance = find_distance(odom_data.pose.pose, target_pose)
        bot_vel.linear.x = C.KP_LIN * distance

    return bot_vel


def calculate_velocity_scan(scan_data: LaserScan, target_angle: int) -> Twist:
    bot_vel = Twist()
    scan_lower_bound, scan_upper_bound = wrap_bounds(
        target_angle, 360, C.FRONT_ANGLE_RANGE)
    scan_ranges = np.array(scan_data.ranges)

    if (scan_lower_bound > scan_upper_bound):
        scan_ranges[np.arange(scan_upper_bound, scan_lower_bound).astype(
            int)] = C.DETECTION_LIMIT
    else:
        scan_ranges[np.arange(scan_lower_bound, scan_upper_bound).astype(
            int)] = C.DETECTION_LIMIT

    if np.isinf(np.max(scan_ranges)):
        distance_to_object = np.min(scan_ranges)
        if distance_to_object == C.DETECTION_LIMIT:
            return None
        object_angle = np.amin(scan_ranges)

    else:
        distance_to_object = scan_ranges[target_angle]
        object_angle = np.unwrap(np.deg2rad(target_angle))

    bot_vel.linear.x = (
        min(C.KP_LIN * distance_to_object, C.KP_LIN))
    bot_vel.angular.z = C.KP_ANG * object_angle * C.LOCK_ON_MODIFIER_FOLLOW

    return bot_vel


def calculate_velocity_img_cen(img_cen: ImgCen) -> Twist:
    bot_vel = Twist()
    if img_cen.target != C.TARGET_NONE:
        bot_vel.angular.z = C.KP_ANG * -img_cen.center_x * C.LOCK_ON_MODIFIER_APPROACH
    else:
        bot_vel.angular.z = C.KP_ANG * C.SEARCH_TURN_VEL
    return bot_vel


class MovementController(object):

    def __init__(self):
        rospy.init_node('q_bot_movement')

        self.current_state = C.MOVEMENT_STATE_IDLE
        self.starting_pose = Pose()
        self.last_twist = Twist()
        self.initialized = False

        self.publishers = self.initialize_publishers()
        self.initialize_subscribers()

    def initialize_publishers(self) -> dict:
        publishers = {}
        publishers[C.CMD_VEL_TOPIC] = rospy.Publisher(
            C.CMD_VEL_TOPIC, Twist, queue_size=C.QUEUE_SIZE
        )
        return publishers

    def initialize_subscribers(self) -> None:
        rospy.Subscriber(C.ODOM_TOPIC, Odometry, self.process_odom)
        rospy.Subscriber(C.SCAN_TOPIC, LaserScan, self.process_scan)
        rospy.Subscriber(C.IMG_CEN_TOPIC, ImgCen, self.process_img_cen)
        rospy.Subscriber(
            C.ACTION_STATE_TOPIC,
            ActionState,
            self.process_action_state)

    def set_state(self, state: str) -> None:
        self.current_state = state

    def process_action_state(self, action_state: ActionState) -> None:
        new_state = action_state.action_state

        if new_state == C.ACTION_STATE_IDLE:
            self.set_state(C.MOVEMENT_STATE_IDLE)

        elif new_state == C.ACTION_STATE_MOVE_CENTER:
            self.set_state(C.MOVEMENT_STATE_GO_TO_POSITION)

        elif new_state == C.ACTION_STATE_LOCATE_DUMBBELL:
            self.set_state(C.MOVEMENT_STATE_FIND_OBJECT)

        elif new_state == C.ACTION_STATE_MOVE_DUMBBELL:
            self.set_state(C.MOVEMENT_STATE_FOLLOW_OBJECT)

        elif new_state == C.ACTION_STATE_GRAB:
            self.set_state(C.MOVEMENT_STATE_APPROACH_OBJECT)

        elif new_state == C.ACTION_STATE_LOCATE_BLOCK:
            self.set_state(C.MOVEMENT_STATE_FIND_OBJECT)

        elif new_state == C.ACTION_STATE_MOVE_BLOCK:
            self.set_state(C.MOVEMENT_STATE_FOLLOW_OBJECT)

        elif new_state == C.ACTION_STATE_RELEASE:
            self.set_state(C.MOVEMENT_STATE_APPROACH_OBJECT)

    def process_odom(self, odom_data: Odometry) -> None:
        if not self.initialized:
            self.starting_pose = odom_data.pose.pose
            self.initialized = True
        else:
            if self.current_state == C.MOVEMENT_STATE_GO_TO_POSITION:
                bot_vel = calculate_velocity_odom(
                    odom_data, self.starting_pose)
                self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)
                self.last_twist = bot_vel

    def process_scan(self, scan_data: LaserScan) -> None:
        if self.current_state == C.MOVEMENT_STATE_FOLLOW_OBJECT:
            bot_vel = calculate_velocity_scan(scan_data, 0)
            if bot_vel is not None:
                self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)
                self.last_twist = bot_vel

    def process_img_cen(self, img_cen: ImgCen) -> None:
        if self.current_state == C.MOVEMENT_STATE_FIND_OBJECT:
            bot_vel = calculate_velocity_img_cen(img_cen)
            self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)
            self.last_twist = bot_vel

        elif self.current_state == C.MOVEMENT_STATE_FOLLOW_OBJECT:
            bot_vel = calculate_velocity_img_cen(img_cen)
            bot_vel.linear.x = self.last_twist.linear.x
            self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)
            self.last_twist = bot_vel

        elif self.current_state == C.MOVEMENT_STATE_APPROACH_OBJECT:
            bot_vel = calculate_velocity_img_cen(img_cen)
            bot_vel.linear.x = C.APPROACH_SPEED
            self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)
            self.last_twist = bot_vel

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    controller = MovementController()
    controller.run()
