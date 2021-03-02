#!/usr/bin/env python3

import rospy
import constants as C
from q_learning_project.msg import RobotMoveDBToBlock
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node('q_bot_tester')

    publisher = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10, latch=True)

    rospy.sleep(1)

    action = RobotMoveDBToBlock()
    action.robot_db = "red"
    action.block_id = 1

    publisher.publish(action)
    print("Published")