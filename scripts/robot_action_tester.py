#!/usr/bin/env python3

"""
A node used to test modules by publishing messages.
"""

import rospy
import constants as C
from q_learning_project.msg import ManipulatorAction, ActionState

if __name__ == "__main__":
    rospy.init_node('q_bot_tester')

    publisher = rospy.Publisher(
        C.MANIPULATOR_ACTION_TOPIC, ManipulatorAction, queue_size=10, latch=True)

    rospy.sleep(1)

    action = ManipulatorAction()
    action.is_confirmation = False
    action.robot_db = "red"
    action.block_id = 27

    # publisher = rospy.Publisher(
    #     C.ACTION_STATE_TOPIC, ActionState, queue_size=10, latch=True)

    # rospy.sleep(1)

    # action = ActionState()
    # action.action_state = C.ACTION_STATE_MOVE_CENTER
    # action.robot_db = "red"
    # action.block_id = 1

    publisher.publish(action)
    print("Published")
