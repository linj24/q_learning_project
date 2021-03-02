#!/usr/bin/env python3

import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from q_learning_project.msg import ActionState, ArmRaised
import constants as C


def create_JointState(joint_names, joint_positions):
    joint_state = JointState()
    joint_state.name = joint_names
    joint_state.position = joint_positions
    return joint_state


class ArmController(object):

    def __init__(self):
        rospy.init_node('q_arm')

        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander(
            "gripper")
        self.current_state = C.ARM_STATE_IDLE

        self.publishers = self.initialize_publishers()
        self.initialize_subscribers()

    def initialize_publishers(self) -> dict:
        publishers = {}
        publishers[C.ARM_RAISED_TOPIC] = rospy.Publisher(
            C.ARM_RAISED_TOPIC, ArmRaised, queue_size=C.QUEUE_SIZE
        )
        return publishers

    def initialize_subscribers(self) -> None:
        rospy.Subscriber(
            C.ACTION_STATE_TOPIC,
            ActionState,
            self.process_action_state)

    def set_state(self, state: str) -> None:
        if state == C.ARM_STATE_UP and self.current_state != C.ARM_STATE_UP:
            self.raise_arm()
            self.close_gripper()
        elif state == C.ARM_STATE_DOWN and self.current_state != C.ARM_STATE_DOWN:
            self.lower_arm()
            self.open_gripper()
        elif state == C.ARM_STATE_GRABBING and self.current_state != C.ARM_STATE_GRABBING:
            self.close_gripper()
            self.raise_arm()
        elif state == C.ARM_STATE_RELEASING and self.current_state != C.ARM_STATE_RELEASING:
            self.lower_arm()
            self.open_gripper()
        self.current_state = state

    def raise_arm(self) -> None:
        self.move_group_arm.go(
            joints=create_JointState(
                C.ARM_JOINT_NAMES,
                C.ARM_JOINT_GOAL_UP),
            wait=True)
        self.move_group_arm.stop()
        arm_raised_flag = ArmRaised()
        arm_raised_flag.arm_raised = True
        self.publishers[C.ARM_RAISED_TOPIC].publish(arm_raised_flag)

    def lower_arm(self) -> None:
        self.move_group_arm.go(
            joints=create_JointState(
                C.ARM_JOINT_NAMES,
                C.ARM_JOINT_GOAL_DOWN),
            wait=True)
        self.move_group_arm.stop()
        arm_raised_flag = ArmRaised()
        arm_raised_flag.arm_raised = False
        self.publishers[C.ARM_RAISED_TOPIC].publish(arm_raised_flag)

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

    def process_action_state(self, action_state):
        new_state = action_state.action_state

        if new_state == C.ACTION_STATE_IDLE:
            self.set_state(C.ARM_STATE_IDLE)

        elif new_state == C.ACTION_STATE_MOVE_CENTER:
            self.set_state(C.ARM_STATE_UP)

        elif new_state == C.ACTION_STATE_LOCATE_DUMBBELL:
            self.set_state(C.ARM_STATE_DOWN)

        elif new_state == C.ACTION_STATE_MOVE_DUMBBELL:
            self.set_state(C.ARM_STATE_DOWN)

        elif new_state == C.ACTION_STATE_GRAB:
            self.set_state(C.ARM_STATE_GRABBING)

        elif new_state == C.ACTION_STATE_LOCATE_BLOCK:
            self.set_state(C.ARM_STATE_UP)

        elif new_state == C.ACTION_STATE_MOVE_BLOCK:
            self.set_state(C.ARM_STATE_UP)

        elif new_state == C.ACTION_STATE_RELEASE:
            self.set_state(C.ARM_STATE_RELEASING)

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    controller = ArmController()
    controller.run()
