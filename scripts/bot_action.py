#!/usr/bin/env python3


import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan
from arm_manipulation import ArmController
from movement import MovementController
from vision import VisionController
import constants as C


class ActionController(object):
    
    def __init__(self):
        rospy.init_node('q_bot_action')

        self.pubs = self.initialize_publishers()
        self.initialize_subscribers()

        self.arm_controller = ArmController()
        self.movement_controller = MovementController()
        self.vision_controller = VisionController()

        self.current_state = C.ACTION_STATE_IDLE
        self.conditions = {
            "IN_CENTER": True,
            "HOLDING_DUMBBELL": False,
            "FACING_DUMBBELL": False,
            "IN_FRONT_OF_DUMBBELL": False,
            "FACING_BLOCK": False,
            "IN_FRONT_OF_BLOCK": False
        }
        
        
    def initialize_publishers(self):
        publishers = {}

        publishers[C.ACTION_STATE_TOPIC] = rospy.Publisher(
            C.ACTION_STATE_TOPIC, int, queue_size=C.QUEUE_SIZE
        )
        publishers[C.ROBOT_ACTION_TOPIC] = rospy.Publisher(
            C.ROBOT_ACTION_TOPIC, RobotMoveDBToBlock, queue_size=C.QUEUE_SIZE
        )

        return publishers


    def initialize_subscribers(self):
        rospy.Subscriber(C.ACTION_STATE_TOPIC, int, self.update_state)


    def get_next_state(self):
        if self.current_state == C.ACTION_STATE_IDLE:
            return C.ACTION_STATE_MOVE_CENTER

        elif self.current_state == C.ACTION_STATE_MOVE_CENTER:
            if (self.conditions["IN_CENTER"] and
                    not self.conditions["HOLDING_DUMBBELL"]):
                return C.ACTION_STATE_LOCATE_DUMBBELL

            elif (self.conditions["IN_CENTER"] and
                    self.conditions["HOLDING_DUMBBELL"]):
                return C.ACTION_STATE_LOCATE_BLOCK

        elif self.current_state == C.ACTION_STATE_LOCATE_DUMBBELL:
            if self.conditions["FACING_DUMBBELL"]:
                return C.ACTION_STATE_MOVE_DUMBBELL

        elif self.current_state == C.ACTION_STATE_MOVE_DUMBBELL:
            if self.conditions["IN_FRONT_OF_DUMBBELL"]:
                return C.ACTION_STATE_GRAB

            elif (not self.conditions["IN_FRONT_OF_DUMBBELL"] and
                    not self.conditions["FACING_DUMBBELL"]):
                return C.ACTION_STATE_LOCATE_DUMBBELL

        elif self.current_state == C.ACTION_STATE_GRAB:
            if self.conditions["HOLDING_DUMBBELL"]:
                return C.ACTION_STATE_MOVE_CENTER
            
        elif self.current_state == C.ACTION_STATE_LOCATE_BLOCK:
            if self.conditions["FACING_BLOCK"]:
                return C.ACTION_STATE_MOVE_BLOCK

        elif self.current_state == C.ACTION_STATE_MOVE_BLOCK:
            if self.conditions["IN_FRONT_OF_BLOCK"]:
                return C.ACTION_STATE_RELEASE

            elif (not self.conditions["IN_FRONT_OF_BLOCK"] and
                    not self.conditions["FACING_BLOCK"]):
                return C.ACTION_STATE_LOCATE_BLOCK

        elif self.current_state == C.ACTION_STATE_RELEASE:
            if not self.conditions["HOLDING_DUMBBELL"]:
                return C.ACTION_STATE_IDLE

        return self.current_state

    
    def set_search_color(self, search_color):
        self.vision_controller.set_state(
            C.VISION_STATE_COLOR_SEARCH, search_color)


    def set_search_number(self, search_number):
        pass

    def update_state(self, new_state):
        pass





    def run(self):
        rospy.spin()

