#!/usr/bin/env python3

"""
Controller object handling all wheel movement based on the robot state.
"""

import rospy
import numpy as np
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import constants as C
from utils import (find_angle_offset, find_distance,
                   wrap_bounds, get_closest_distance_and_angle, round_magnitude)
from q_learning_project.msg import ActionState, ImgCen


class MovementController():
    """
    Tracks the robot's current state and moves the wheels when necessary.
    """

    def __init__(self):
        rospy.init_node('q_bot_movement')

        self.current_state = C.MOVEMENT_STATE_IDLE
        self.starting_pose = Pose()
        self.last_twist = Twist()
        self.search_direction = C.TURN_LEFT
        self.initialized = False

        self.publishers = self.initialize_publishers()
        self.initialize_subscribers()

    def initialize_publishers(self) -> dict:
        """
        Initialize all the publishers this node will use.
        """
        publishers = {}
        publishers[C.CMD_VEL_TOPIC] = rospy.Publisher(
            C.CMD_VEL_TOPIC, Twist, queue_size=C.QUEUE_SIZE
        )
        return publishers

    def initialize_subscribers(self) -> None:
        """
        Initialize all the subscribers this node will use.
        """
        rospy.Subscriber(C.ODOM_TOPIC, Odometry, self.process_odom)
        rospy.Subscriber(C.SCAN_TOPIC, LaserScan, self.process_scan)
        rospy.Subscriber(C.IMG_CEN_TOPIC, ImgCen, self.process_img_cen)
        rospy.Subscriber(
            C.ACTION_STATE_TOPIC,
            ActionState,
            self.process_action_state)

    def set_state(self, state: str) -> None:
        """
        Set the current movement state.
        """
        self.current_state = state

    def calculate_velocity_odom(self, odom_data: Odometry) -> Twist:
        """
        Calculate a velocity for the robot to navigate to the center of the map.
        """
        bot_vel = Twist()

        angle_offset = find_angle_offset(
            odom_data.pose.pose, self.starting_pose)
        bot_vel.angular.z = C.KP_ANG * -angle_offset

        if abs(angle_offset) < (np.pi / 4):
            distance = find_distance(odom_data.pose.pose, self.starting_pose)
            bot_vel.linear.x = C.KP_LIN * distance

        return bot_vel

    def calculate_velocity_scan(self, scan_data: LaserScan, include_linear: bool) -> Twist:
        """
        Calculate a linear velocity for the robot from a forward-facing laser scan.
        """
        bot_vel = Twist()
        scan_ranges = np.array(scan_data.ranges)
        if not np.isinf(scan_ranges[0]):
            distance_to_object = scan_ranges[0]
            angle_to_object = 0
        else:
            distance_to_object, angle_to_object = get_closest_distance_and_angle(
                scan_data)

            if np.isinf(distance_to_object):
                distance_to_object = 0

            if angle_to_object > 180:
                angle_to_object = angle_to_object - 360

        if include_linear:
            bot_vel.linear.x = C.KP_LIN * distance_to_object
        # if angle_to_object > 0:
        #     self.search_direction = C.TURN_LEFT
        # else:
        #     self.search_direction = C.TURN_RIGHT
        # , C.KP_ANG / C.LOCK_ON_MODIFIER_FOLLOW)
        bot_vel.angular.z = C.KP_ANG * np.deg2rad(angle_to_object)
        return bot_vel

    def process_action_state(self, action_state: ActionState) -> None:
        """
        Receive an action state and update the movement state accordingly.
        """
        new_state = action_state.action_state

        if new_state == C.ACTION_STATE_IDLE:
            self.set_state(C.MOVEMENT_STATE_IDLE)

        elif new_state == C.ACTION_STATE_MOVE_CENTER:
            self.set_state(C.MOVEMENT_STATE_GO_TO_POSITION)

        elif new_state == C.ACTION_STATE_LOCATE_DUMBBELL:
            self.set_state(C.MOVEMENT_STATE_FIND_OBJECT)

        elif new_state == C.ACTION_STATE_CENTER_DUMBBELL:
            self.set_state(C.MOVEMENT_STATE_CENTER_OBJECT)

        elif new_state == C.ACTION_STATE_WAIT_FOR_COLOR_IMG:
            self.set_state(C.MOVEMENT_STATE_WAIT_FOR_IMG)

        elif new_state == C.ACTION_STATE_MOVE_DUMBBELL:
            self.set_state(C.MOVEMENT_STATE_FOLLOW_OBJECT)

        elif new_state == C.ACTION_STATE_GRAB:
            self.set_state(C.MOVEMENT_STATE_APPROACH_OBJECT)

        elif new_state == C.ACTION_STATE_LOCATE_BLOCK:
            self.set_state(C.MOVEMENT_STATE_FIND_OBJECT)

        elif new_state == C.ACTION_STATE_CENTER_BLOCK:
            self.set_state(C.MOVEMENT_STATE_CENTER_OBJECT)

        elif new_state == C.ACTION_STATE_WAIT_FOR_NUMBER_IMG:
            self.set_state(C.MOVEMENT_STATE_WAIT_FOR_IMG)

        elif new_state == C.ACTION_STATE_MOVE_BLOCK:
            self.set_state(C.MOVEMENT_STATE_FOLLOW_OBJECT)

        elif new_state == C.ACTION_STATE_RELEASE:
            self.set_state(C.MOVEMENT_STATE_APPROACH_OBJECT)

    def process_odom(self, odom_data: Odometry) -> None:
        """
        Receive an odometry reading and calculate a velocity
        if the robot is moving to the center of the map.
        """
        if not self.initialized:
            self.starting_pose = odom_data.pose.pose
            self.initialized = True
        else:
            if self.current_state == C.MOVEMENT_STATE_GO_TO_POSITION:
                bot_vel = self.calculate_velocity_odom(
                    odom_data)
                self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)

    def process_scan(self, scan_data: LaserScan) -> None:
        """
        Receive a laser scan reading and calculate a velocity
        if the robot is approaching an object.
        """
        bot_vel = Twist()
        if self.current_state == C.MOVEMENT_STATE_FIND_OBJECT:
            bot_vel.angular.z = C.SEARCH_TURN_VEL * self.search_direction
            self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)
        elif self.current_state == C.MOVEMENT_STATE_CENTER_OBJECT:
            bot_vel = self.calculate_velocity_scan(scan_data, False)
            self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)
        elif self.current_state == C.MOVEMENT_STATE_FOLLOW_OBJECT:
            bot_vel = self.calculate_velocity_scan(scan_data, True)
            self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)
        elif self.current_state == C.MOVEMENT_STATE_WAIT_FOR_IMG:
            self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)
        elif self.current_state == C.MOVEMENT_STATE_APPROACH_OBJECT:
            bot_vel.linear.x = C.APPROACH_SPEED
            self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)

    def process_img_cen(self, img_cen: ImgCen) -> None:
        """
        Receive coordinates for the center of an image and
        calculate a velocity if the robot is tracking an object.
        """
        if (self.current_state == C.MOVEMENT_STATE_FIND_OBJECT or
                self.current_state == C.MOVEMENT_STATE_FOLLOW_OBJECT or
                self.current_state == C.MOVEMENT_STATE_APPROACH_OBJECT):
            if img_cen.target != C.TARGET_NONE:
                if img_cen.center_x < 0:
                    self.search_direction = C.TURN_LEFT
                else:
                    self.search_direction = C.TURN_RIGHT

    def run(self) -> None:
        """
        Listen for incoming messages.
        """
        rospy.spin()


if __name__ == "__main__":
    controller = MovementController()
    controller.run()
