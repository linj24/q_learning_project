#!/usr/bin/env python3

import rospy
import numpy as np
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan
import constants as C
from utils import find_angle_offset, find_distance, wrap_bounds, ImgCentroidMsg


def calculate_velocity_odom(bot_pose, target_pose):
    bot_vel = Twist()
    
    angle_offset = find_angle_offset(bot_pose, target_pose)
    bot_vel.angular.z = C.KP_ANG * angle_offset

    if abs(angle_offset) < 90:
        distance = find_distance(bot_pose, target_pose)
        bot_vel.linear.x = C.KP_LIN * distance

    return bot_vel


def calculate_velocity_scan(scan_data, target_angle):
    bot_vel = Twist()
    scan_lower_bound, scan_upper_bound = wrap_bounds(
        target_angle, 360, C.FRONT_ANGLE_RANGE)
    scan_ranges = np.array(scan_data.ranges)

    if (scan_lower_bound > scan_upper_bound):
        scan_ranges[scan_upper_bound:scan_lower_bound] = C.DETECTION_LIMIT
    else:
        scan_ranges[scan_lower_bound:scan_upper_bound] = C.DETECTION_LIMIT
    
    if np.isinf(np.max(scan_ranges)):
        distance_to_object = np.min(scan_ranges)
        if distance_to_object == C.DETECTION_LIMIT:
            return None
        object_angle = np.amin(scan_ranges)
        
    else:
        distance_to_object = scan_ranges[target_angle]
        object_angle = np.unwrap(np.deg2rad(target_angle))

    bot_vel.linear.x = C.KP_LIN * distance_to_object
    bot_vel.angular.z = C.KP_ANG * object_angle

    return bot_vel


def calculate_velocity_img_cen(centroid):
    bot_vel = Twist()
    if centroid.centroid is not None:
        bot_vel.angular.z = C.KP_ANG * centroid.centroid[1]
    else:
        bot_vel.angular.z = C.KP_ANG * C.SEARCH_TURN_VEL
    return bot_vel


class MovementController(object):
    
    def __init__(self):
        rospy.init_node('q_bot_movement')

        self.publishers = self.initialize_publishers()
        self.initialize_subscribers()

        self.current_state = C.MOVEMENT_STATE_IDLE
        self.starting_pose = Pose()
        self.last_twist = Twist()
        
        
    def initialize_publishers(self):
        publishers = {}
        publishers[C.CMD_VEL_TOPIC] = rospy.Publisher(
            C.CMD_VEL_TOPIC, Twist, queue_size=C.QUEUE_SIZE
        )
        return publishers


    def initialize_subscribers(self):
        rospy.Subscriber(C.ODOM_TOPIC, Pose, self.process_odom)
        rospy.Subscriber(C.SCAN_TOPIC, LaserScan, self.process_scan)
        rospy.Subscriber(C.IMG_CEN_TOPIC, ImgCentroidMsg, self.process_img_cen)


    def set_starting_pose(self, pose):
        self.starting_pose = pose


    def set_state(self, state):
        self.current_state = state


    def process_odom(self, bot_pose):
        if self.current_state == C.MOVEMENT_STATE_GO_TO_POSITION:
            bot_vel = calculate_velocity_odom(bot_pose, self.starting_pose)
            self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)
            self.last_twist = bot_vel


    def process_scan(self, scan_data):
        if self.current_state == C.MOVEMENT_STATE_FOLLOW_OBJECT:
            bot_vel = calculate_velocity_scan(scan_data, 0)
            if bot_vel is not None:
                self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)
                self.last_twist = bot_vel


    def process_img_cen(self, img_cen):
        if self.current_state == C.MOVEMENT_STATE_FIND_OBJECT:
            bot_vel = calculate_velocity_img_cen(img_cen)
            self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)
            self.last_twist = bot_vel
                
        elif self.current_state == C.MOVEMENT_STATE_FOLLOW_OBJECT:
            bot_vel = calculate_velocity_img_cen(img_cen)
            bot_vel.linear.x = self.last_twist.linear.x
            self.publishers[C.CMD_VEL_TOPIC].publish(bot_vel)
            self.last_twist = bot_vel


    def run(self):
        rospy.spin()

