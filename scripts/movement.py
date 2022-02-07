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
from utils import (yaw_from_quaternion, find_angle_offset, find_distance,
                   get_closest_distance_and_angle, round_magnitude)
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
        self.current_yaw = 0
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
        if state != self.current_state:
            rospy.loginfo(f"[Movement] Current state is {state}")
        self.current_state = state

    def calculate_velocity_odom(self, odom_data: Odometry) -> Twist:
        """
        Calculate a velocity for the robot to navigate to the center of the map.
        """
        bot_vel = Twist()

        # Calculate the angular displacement from the bot's current orientation
        # to where its original spawn point was
        angle_offset = find_angle_offset(
            odom_data.pose.pose, self.starting_pose)
        bot_vel.angular.z = C.KP_ANG * angle_offset

        if abs(angle_offset) < (np.pi / 4):
            # Only move forward if the bot is facing the spawn point
            distance = find_distance(odom_data.pose.pose, self.starting_pose)
            bot_vel.linear.x = C.KP_LIN * distance

        return bot_vel

    def calculate_velocity_scan(
            self,
            scan_data: LaserScan,
            front_angle_range: int) -> Twist:
        """
        Calculate a linear velocity for the robot from a forward-facing laser scan.
        """
        bot_vel = Twist()

        # If the bot is facing an object, get the distance to the closest part
        # of that object
        distance_to_object, _ = get_closest_distance_and_angle(
            scan_data, front_angle_range)

        if np.isinf(distance_to_object):
            distance_to_object = 0

        # Scale the bot's linear velocity by how close it is
        bot_vel.linear.x = C.KP_LIN * distance_to_object
        bot_vel.angular.z = self.last_twist.angular.z

        return bot_vel

    def calculate_velocity_img_cen(
            self,
            img_cen_data: ImgCen) -> Twist:
        """
        Calculate an angular velocity for the robot from an image scan.
        """
        bot_vel = Twist()

        if img_cen_data.target != C.TARGET_NONE:
            # If the scan found what the bot was looking for
            bot_vel.angular.z = -round_magnitude(C.KP_ANG * C.IMG_CEN_ANGLE_MODIFIER * img_cen_data.center_x, C.MIN_ANG_VEL)
        
        bot_vel.linear.x = self.last_twist.linear.x

        return bot_vel

    def process_action_state(self, action_state: ActionState) -> None:
        """
        Receive an action state and update the movement state accordingly.
        """
        new_state = action_state.action_state

        # Process messages from the main controller re: robot state
        if new_state == C.ACTION_STATE_IDLE:
            self.set_state(C.MOVEMENT_STATE_IDLE)

        elif new_state == C.ACTION_STATE_MOVE_CENTER:
            self.set_state(C.MOVEMENT_STATE_GO_TO_POSITION)

        elif new_state == C.ACTION_STATE_LOCATE_DUMBBELL:
            self.set_state(C.MOVEMENT_STATE_FIND_OBJECT)

        elif new_state == C.ACTION_STATE_CENTER_DUMBBELL:
            self.set_state(C.MOVEMENT_STATE_CENTER_OBJECT)

        elif new_state == C.ACTION_STATE_MOVE_DUMBBELL:
            self.set_state(C.MOVEMENT_STATE_FOLLOW_OBJECT)

        elif new_state == C.ACTION_STATE_GRAB:
            self.set_state(C.MOVEMENT_STATE_APPROACH_OBJECT)

        elif new_state == C.ACTION_STATE_LOCATE_BLOCK:
            self.set_state(C.MOVEMENT_STATE_FIND_OBJECT)

        elif new_state == C.ACTION_STATE_CENTER_BLOCK:
            self.set_state(C.MOVEMENT_STATE_CENTER_OBJECT)

        elif new_state == C.ACTION_STATE_MOVE_BLOCK:
            self.set_state(C.MOVEMENT_STATE_FOLLOW_OBJECT)

        elif new_state == C.ACTION_STATE_RELEASE:
            self.set_state(C.MOVEMENT_STATE_APPROACH_OBJECT)

        elif new_state == C.ACTION_STATE_BACK_AWAY:
            self.set_state(C.MOVEMENT_STATE_BACK_AWAY)

    def process_odom(self, odom_data: Odometry) -> None:
        """
        Receive an odometry reading and calculate a velocity
        if the robot is moving to the center of the map.
        """
        if not self.initialized:
            # Set the starting pose when the bot first spawns in
            self.starting_pose = odom_data.pose.pose
            self.initialized = True
        else:
            # Save the current yaw to check for blocks on the side
            self.current_yaw = yaw_from_quaternion(
                odom_data.pose.pose.orientation)

            if self.current_state == C.MOVEMENT_STATE_GO_TO_POSITION:
                # If the bot needs to move to the center, calculate a velocity
                # and publish it
                bot_vel = self.calculate_velocity_odom(
                    odom_data)
                self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)
                self.last_twist = bot_vel

    def process_scan(self, scan_data: LaserScan) -> None:
        """
        Receive a laser scan reading and calculate a velocity
        if the robot is approaching an object.
        """
        bot_vel = None
        if self.current_state == C.MOVEMENT_STATE_IDLE:
            # If the bot isn't doing anything, stop it from moving
            bot_vel = Twist()
        if self.current_state == C.MOVEMENT_STATE_FIND_OBJECT:
            # If the bot is searching for an object, set it to rotate slowly
            # in the direction it last saw something
            bot_vel = Twist()
            bot_vel.angular.z = C.SEARCH_TURN_VEL * self.search_direction
        elif self.current_state == C.MOVEMENT_STATE_CENTER_OBJECT:
            # Defer lock-on to the camera
            pass
        elif self.current_state == C.MOVEMENT_STATE_FOLLOW_OBJECT:
            # If the bot has locked on to an object and needs to move
            # in front of it, use proportional control to approach it
            bot_vel = self.calculate_velocity_scan(
                scan_data, C.LOCK_ON_RANGE)
        elif self.current_state == C.MOVEMENT_STATE_APPROACH_OBJECT:
            # If the bot is trying to pick up or release a dumbbell, slowly
            # slowly move forward to make contact
            bot_vel = Twist()
            bot_vel.linear.x = C.APPROACH_SPEED
        elif self.current_state == C.MOVEMENT_STATE_BACK_AWAY:
            # If the bot has released a dumbbell, back away to unhook the
            # dumbbell and give the bot room to maneuver
            bot_vel = Twist()
            bot_vel.linear.x = C.BACK_AWAY_SPEED

        if bot_vel is not None:
            self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)
            self.last_twist = bot_vel

    def process_img_cen(self, img_cen: ImgCen) -> None:
        """
        Receive coordinates for the center of an image and
        calculate a velocity if the robot is tracking an object.
        """
        bot_vel = None

        if self.current_state == C.MOVEMENT_STATE_CENTER_OBJECT or self.current_state == C.MOVEMENT_STATE_FOLLOW_OBJECT:
            bot_vel = self.calculate_velocity_img_cen(img_cen)
        
        if bot_vel is not None:
            self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)
            self.last_twist = bot_vel

    def run(self) -> None:
        """
        Listen for incoming messages.
        """
        rospy.spin()


if __name__ == "__main__":
    controller = MovementController()
    controller.run()
