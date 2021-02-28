#!/usr/bin/env python3

import rospy, sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import constants as C

class ArmController(object):
    
    def __init__(self):
        #rospy.init_node('q_arm')

        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        self.current_state = C.ARM_STATE_IDLE


    def set_state(self, state: str) -> None:
        self.current_state = state
        if state == C.ARM_STATE_UP:
            self.raise_arm()
        elif state == C.ARM_STATE_DOWN:
            self.lower_arm()
        elif state == C.ARM_STATE_GRABBING:
            self.close_gripper()
            self.raise_arm()
        elif state == C.ARM_STATE_RELEASING:
            self.lower_arm()
            self.open_gripper()


    def raise_arm(self) -> None:
        self.move_group_arm.go(C.ARM_JOINT_GOAL_UP, wait=True)
        self.move_group_arm.stop()


    def lower_arm(self) -> None:
        self.move_group_arm.go(C.ARM_JOINT_GOAL_DOWN, wait=True)
        self.move_group_arm.stop()


    def close_gripper(self) -> None:
        self.move_group_gripper.go(C.GRIPPER_JOINT_GOAL_CLOSED, wait=True)
        self.move_group_gripper.stop()


    def open_gripper(self) -> None:
        self.move_group_gripper.go(C.GRIPPER_JOINT_GOAL_OPEN, wait=True)
        self.move_group_gripper.stop()