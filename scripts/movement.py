#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan
from q_learning import RobotMoveDBToBlock

CMD_VEL_TOPIC = "cmd_vel"
SCAN_TOPIC = "scan"
QUEUE_SIZE = 10

STATE_IDLE = 0
STATE_SCOUT = 1
STATE_MOVE_D = 2
STATE_GRAB = 3
STATE_MOVE_B = 4
STATE_RELEASE = 5

MVMT_THRESH_LIN = 0.2
MVMT_THRESH_ANG = 0.6

class MovementController(object):
    
    def __init__(self):
        rospy.init_node('q_bot_movement')

        self.pubs = initialize_publishers()
        initialize_subscribers()
        
        
    def initialize_publishers(self):
        publishers = {}
        return publishers


    def initialize_subscribers(self):
        rospy.Subscriber(CMD_VEL_TOPIC, int, self.update_state)


    def update_state(self, new_state):
        pass


    def run(self):
        rospy.spin()

