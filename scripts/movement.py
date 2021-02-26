#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan
import constants as C


def calculate_velocity_odom(bot_pose, target_pose):
    bot_vel = Twist()
    
    angle_offset = find_angle_offset(bot_pose, target_pose)
    bot_vel.angular.z = KP_ANG * angle_offset

    if abs(angle_offset) < 90:
        distance_offset = find_distance_offset(
            bot_pose, target_pose)
        distance = distance_offset[0]**2 + distance_offset[1]**2
        bot_vel.linear.x = KP_LIN * distance

    return bot_vel


def calculate_velocity_scan(scan_data, target_angle):
    bot_vel = Twist()


class MovementController(object):
    
    def __init__(self):
        rospy.init_node('q_bot_movement')

        self.pubs = initialize_publishers()
        initialize_subscribers()

        self.current_state = MOVEMENT_STATE_IDLE
        self.starting_pose = Pose()
        self.last_twist = Twist()
        
        
    def initialize_publishers(self):
        publishers = {}
        publishers[CMD_VEL_TOPIC] = rospy.Publisher(
            CMD_VEL_TOPIC, Twist, queue_size=QUEUE_SIZE
        )
        return publishers


    def initialize_subscribers(self):
        rospy.Subscriber(ACTION_STATE_TOPIC, str, self.process_state)
        rospy.Subscriber(ODOM_TOPIC, Pose, self.process_odom)
        rospy.Subscriber(SCAN_TOPIC, LaserScan, self.process_scan)
        rospy.Subscriber(IMG_CEN_TOPIC, ImgCentroidMsg, self.process_img_cen)


    def set_starting_pose(self, pose):
        self.starting_pose = pose


    def process_odom(self, bot_pose):
        if self.current_state == C.MOVEMENT_STATE_GO_TO_POSITION:
            bot_vel = calculate_velocity_odom(bot_pose, self.starting_pose)
            publishers[CMD_VEL_TOPIC].publish(bot_vel)


    def process_scan(self, scan_data):
        if self.current_state == MOVEMENT_STATE_FOLLOW_OBJECT:
            pass


    def process_img_cen(self, img_cen):
        if self.state == ACTION_STATE_LOCATE_DUMBBELL:
            pass
        elif self.state == ACTION_STATE_LOCATE_BLOCK:
            pass

    def update_state(self, new_state):
        pass


    def run(self):
        rospy.spin()

