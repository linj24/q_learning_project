#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan
from q_learning import RobotMoveDBToBlock
from arm import ArmController
from movement import MovementController
from vision import VisionController

ACTION_STATE_TOPIC = "action_state"
ROBOT_ACTION_TOPIC = "q_learning/RobotMoveDBToBlock"
QUEUE_SIZE = 10

STATE_IDLE = 0
STATE_SCOUT = 1
STATE_MOVE_D = 2
STATE_GRAB = 3
STATE_MOVE_B = 4
STATE_RELEASE = 5

MVMT_THRESH_LIN = 0.2
MVMT_THRESH_ANG = 0.6

class ActionController(object):
    
    def __init__(self):
        rospy.init_node('q_bot_action')

        self.pubs = initialize_publishers()
        initialize_subscribers()

        self.arm_controller = ArmController()
        self.movement_controller = MovementController()
        self.vision_controller = VisionController()
        
        
    def initialize_publishers(self):
        publishers = {}

        publishers[ACTION_STATE_TOPIC] = rospy.Publisher(
            ACTION_STATE_TOPIC, int, queue_size=QUEUE_SIZE
        )
        publishers[ROBOT_ACTION_TOPIC] = rospy.Publisher(
            ROBOT_ACTION_TOPIC, RobotMoveDBToBlock, queue_size=QUEUE_SIZE
        )

        return publishers


    def initialize_subscribers(self):
        rospy.Subscriber(ACTION_STATE_TOPIC, int, self.update_state)


    def scout_field(self):
        self.movement_controller.move_to_center()



    def update_state(self, new_state):
        if self.state == STATE_IDLE and new_state == STATE_SCOUT:
            pass





    def run(self):
        rospy.spin()

