#! /usr/bin/python3

import rospy
import moveit_commander
import constants as C
from sensor_msgs.msg import JointState

move_group_arm = moveit_commander.MoveGroupCommander("arm")
move_group_gripper = moveit_commander.MoveGroupCommander(
    "gripper")

joint_goal = move_group_arm.get_current_joint_values()
print(joint_goal)
move_group_arm.set_start_state_to_current_state()
# increasing the tolerance prevents the node from crashing
move_group_arm.set_goal_joint_tolerance(0.1)
# wait=True triggers the ABORTED error
# arm controller is completing execution right when it begins, which is why it's aborting... why is the execution marked as complete?
move_group_arm.go(C.ARM_JOINT_GOAL_DOWN, wait=True)
print("huh?")
#move_group_arm.stop()