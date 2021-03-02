#!/usr/bin/env python3

import rospy, sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from q_learning_project import ActionState
import constants as C



def create_JointState(joint_names, joint_positions):
    joint_state = JointState()
    joint_state.name = joint_names
    joint_state_position = joint_positions
    return joint_state


class ArmController(object):
    
    def __init__(self):
        rospy.init_node('q_arm')

        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        self.current_state = C.ARM_STATE_IDLE

        self.initialize_subscribers()


    def initialize_subscribers(self) -> None:
        rospy.Subscriber(
            C.ACTION_STATE_TOPIC,
            ActionState,
            self.process_action_state)


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
        self.move_group_arm.go(
            joints=create_JointState(
                C.ARM_JOINT_NAMES,
                C.ARM_JOINT_GOAL_UP),
            wait=True)
        self.move_group_arm.stop()


    def lower_arm(self) -> None:
        self.move_group_arm.go(
            joints=create_JointState(
                C.ARM_JOINT_NAMES,
                C.ARM_JOINT_GOAL_DOWN),
            wait=True)
        self.move_group_arm.stop()


    def close_gripper(self) -> None:
        self.move_group_gripper.go(
            joints=create_JointState(
                C.GRIPPER_JOINT_NAMES,
                C.GRIPPER_JOINT_GOAL_CLOSED),
            wait=True)
        self.move_group_gripper.stop()


    def open_gripper(self) -> None:
        self.move_group_gripper.go(
            joints=create_JointState(
                C.GRIPPER_JOINT_NAMES,
                C.GRIPPER_JOINT_GOAL_OPEN),
            wait=True)
        self.move_group_gripper.stop()


    def run(self) -> None:
        rospy.spin()