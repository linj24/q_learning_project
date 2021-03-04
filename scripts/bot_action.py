#!/usr/bin/env python3

"""
Controller object updating the robot state based on sensor readings.
"""

import constants as C
import numpy as np
from utils import find_distance, is_centered
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from q_learning_project.msg import (RobotMoveDBToBlock, ActionState, ImgCen,
                                    ArmRaised, ManipulatorAction)


class ActionController():
    """
    Tracks the robot's current state and changes it when certain conditions have been met.
    """

    def __init__(self):
        rospy.init_node('q_bot_action')

        self.current_state = C.ACTION_STATE_IDLE
        self.starting_pose = Pose()
        self.current_robot_action = RobotMoveDBToBlock()
        self.initialized = False
        self.conditions = {
            "IN_CENTER": True,
            "FACING_UNKNOWN_OBJECT": False,
            # If IN_FRONT_OF_CLOSE_OBJECT is true so will FACING_UNKNOWN_OBJECT, but
            # the reverse is only true if the object we're facing is close
            "IN_FRONT_OF_CLOSE_OBJECT": False,
            "HOLDING_DUMBBELL": False,
            # The below true will only be set when we know what the object being faced is
            "FACING_TARGET": False,
            "FACING_SCANNED_OBJECT": False,
            "FACING_MATCHED_OBJECT": False,
            "WAITING_FOR_IMG": False,
            "NN_RESPONSE_RECEIVED": False,
            "CENTERED": False,
            "HAS_FACED_NOTHING_SINCE_NN_RESPONSE": True
        }

        self.initialize_publishers()
        self.initialize_subscribers()

    def initialize_publishers(self) -> dict:
        """
        Initialize all the publishers this node will use.
        """
        self.publishers = {}
        self.publishers[C.ACTION_STATE_TOPIC] = rospy.Publisher(
            C.ACTION_STATE_TOPIC, ActionState, queue_size=C.QUEUE_SIZE
        )
        self.publishers[C.MANIPULATOR_ACTION_TOPIC] = rospy.Publisher(
            C.MANIPULATOR_ACTION_TOPIC, ManipulatorAction, queue_size=C.QUEUE_SIZE
        )

    def initialize_subscribers(self) -> None:
        """
        Initialize all the subscribers this node will use.
        """
        rospy.Subscriber(C.ODOM_TOPIC, Odometry, self.process_odom)
        rospy.Subscriber(C.SCAN_TOPIC, LaserScan, self.process_scan)
        rospy.Subscriber(C.IMG_CEN_TOPIC, ImgCen, self.process_img_cen)
        rospy.Subscriber(C.MANIPULATOR_ACTION_TOPIC,
                         ManipulatorAction, self.process_manipulator_action)
        rospy.Subscriber(C.ARM_RAISED_TOPIC, ArmRaised,
                         self.process_arm_raised)

    def create_action_state_msg(self) -> ActionState:
        """
        Create an ActionState message from the robot's current state.
        """
        action_state = ActionState()
        action_state.action_state = self.current_state
        action_state.robot_db = self.current_robot_action.robot_db
        action_state.block_id = self.current_robot_action.block_id
        return action_state

    def create_manipulator_action_msg(self) -> ManipulatorAction:
        """
        Create a ManipulatorAction message from the robot's current state.
        """
        confirmation = ManipulatorAction()
        confirmation.is_confirmation = True
        confirmation.block_id = self.current_robot_action.block_id
        confirmation.robot_db = self.current_robot_action.robot_db
        return confirmation

    def process_odom(self, odom_data: Odometry) -> None:
        """
        Receive an odometry reading and check if the robot is in the center of the map.
        """
        if not self.initialized:
            self.starting_pose = odom_data.pose.pose
            self.initialized = True
        else:
            distance = find_distance(
                self.starting_pose, odom_data.pose.pose)
            if distance < C.CENTER_RADIUS:
                self.conditions["IN_CENTER"] = True
            else:
                self.conditions["IN_CENTER"] = False
            self.update_controller_states(self.get_next_state())

    def process_scan(self, scan_data: LaserScan) -> None:
        """
        Receive a scan reading and check if the robot is directly in front of an object.
        """
        front_distance = min(
            np.min(scan_data.ranges[:C.FRONT_ANGLE_RANGE//2]),
            np.min(scan_data.ranges[-C.FRONT_ANGLE_RANGE//2:]))

        self.conditions["FACING_UNKNOWN_OBJECT"] = front_distance < float(
            'inf')
        # x is defined just to make the writing of a large boolean less ugly
        # It is meant to denote if we have faced nothing since last NN response
        # This is needed in the locating object state because right after
        # processing a NN response we are still facing that object but don't want to
        # process the current image, which is technically still facing that object,
        # until we have moved past it (faced nothing)

        # We must either be facing nothing now, or have already set the variable to False
        # in the past
        x = not self.conditions["FACING_UNKNOWN_OBJECT"]
        x = x or self.conditions["HAS_FACED_NOTHING_SINCE_NN_RESPONSE"]
        # We must also not have an unprocessed NN response
        # in which case we would be facing an object so we would not have
        # faced nothing since last NN response
        x = x and not self.conditions["NN_RESPONSE_RECEIVED"]
        self.conditions["HAS_FACED_NOTHING_SINCE_NN_RESPONSE"] = x
        # centered is only used once we know we are facing an unknown object period
        self.conditions["CENTERED"] = is_centered(scan_data)
        self.conditions["IN_FRONT_OF_CLOSE_OBJECT"] = front_distance < C.SAFE_DISTANCE

        self.update_controller_states(self.get_next_state())

    def process_img_cen(self, img_cen_data: ImgCen) -> None:
        """
        Receive an image center and check if the robot is facing an object.
        """
        self.conditions["NN_RESPONSE_RECEIVED"] = True
        if (img_cen_data.target != C.TARGET_NONE and
                abs(img_cen_data.center_x) < C.IMG_CEN_PIXEL_THRESHOLD):
            if (img_cen_data.vision_state == C.VISION_STATE_COLOR_SEARCH or
                    img_cen_data.vision_state == C.VISION_STATE_NUMBER_SEARCH):
                self.conditions["FACING_TARGET"] = True
        else:
            self.conditions["FACING_TARGET"] = False
        self.update_controller_states(self.get_next_state())

    def process_manipulator_action(self, action: ManipulatorAction) -> None:
        """
        Receive an action from the Q learning node and execute it.
        """
        if not action.is_confirmation:
            self.current_robot_action.block_id = action.block_id
            self.current_robot_action.robot_db = action.robot_db
            self.update_controller_states(C.ACTION_STATE_MOVE_CENTER)

    def process_arm_raised(self, raised_flag: ArmRaised) -> None:
        """
        Receive an update from the arm node to check if the arm is currently raised.
        """
        self.conditions["HOLDING_DUMBBELL"] = raised_flag.arm_raised

    def get_next_state(self) -> str:
        """
        Check the current conditions to see if the bot can change state.
        """
        if self.current_state == C.ACTION_STATE_MOVE_CENTER:
            if (self.conditions["IN_CENTER"] and
                    not self.conditions["HOLDING_DUMBBELL"]):
                return C.ACTION_STATE_LOCATE_DUMBBELL

            if (self.conditions["IN_CENTER"] and
                    self.conditions["HOLDING_DUMBBELL"]):
                return C.ACTION_STATE_LOCATE_BLOCK

        elif self.current_state == C.ACTION_STATE_LOCATE_DUMBBELL:
            if self.conditions["FACING_UNKNOWN_OBJECT"] and \
                    self.conditions["HAS_FACED_NOTHING_SINCE_NN_RESPONSE"]:
                if self.conditions["CENTERED"]:
                    return C.ACTION_STATE_WAIT_FOR_COLOR_IMG
                else:
                    return C.ACTION_STATE_CENTER_DUMBBELL

        elif self.current_state == C.ACTION_STATE_CENTER_DUMBBELL:
            if self.conditions["CENTERED"]:
                return C.ACTION_STATE_WAIT_FOR_COLOR_IMG

        elif self.current_state == C.ACTION_STATE_WAIT_FOR_COLOR_IMG:
            if self.conditions["NN_RESPONSE_RECEIVED"]:
                self.conditions["NN_RESPONSE_RECEIVED"] = False
                if self.conditions["FACING_TARGET"]:
                    return C.ACTION_STATE_MOVE_DUMBBELL
                else:
                    return C.ACTION_STATE_LOCATE_DUMBBELL

        elif self.current_state == C.ACTION_STATE_MOVE_DUMBBELL:
            if (self.conditions["IN_FRONT_OF_CLOSE_OBJECT"] and
                    self.conditions["FACING_TARGET"]):
                return C.ACTION_STATE_GRAB

            if not self.conditions["FACING_TARGET"]:
                return C.ACTION_STATE_LOCATE_DUMBBELL

        elif self.current_state == C.ACTION_STATE_GRAB:
            if self.conditions["HOLDING_DUMBBELL"]:
                return C.ACTION_STATE_MOVE_CENTER

        elif self.current_state == C.ACTION_STATE_LOCATE_BLOCK:
            if self.conditions["FACING_UNKNOWN_OBJECT"] and \
                    self.conditions["HAS_FACED_NOTHING_SINCE_NN_RESPONSE"]:
                if self.conditions["CENTERED"]:
                    return C.ACTION_STATE_WAIT_FOR_NUMBER_IMG
                else:
                    return C.ACTION_STATE_CENTER_BLOCK

        elif self.current_state == C.ACTION_STATE_CENTER_BLOCK:
            if self.conditions["CENTERED"]:
                return C.ACTION_STATE_WAIT_FOR_NUMBER_IMG

        elif self.current_state == C.ACTION_STATE_WAIT_FOR_NUMBER_IMG:
            if self.conditions["NN_RESPONSE_RECEIVED"]:
                self.conditions["NN_RESPONSE_RECEIVED"] = False
                if self.conditions["FACING_TARGET"]:
                    return C.ACTION_STATE_MOVE_BLOCK
                else:
                    return C.ACTION_STATE_LOCATE_BLOCK

        elif self.current_state == C.ACTION_STATE_MOVE_BLOCK:
            if (self.conditions["IN_FRONT_OF_CLOSE_OBJECT"] and
                    self.conditions["FACING_TARGET"]):
                return C.ACTION_STATE_RELEASE

            if not self.conditions["FACING_TARGET"]:
                return C.ACTION_STATE_LOCATE_BLOCK

        elif self.current_state == C.ACTION_STATE_RELEASE:
            if not self.conditions["HOLDING_DUMBBELL"]:
                confirmation = self.create_manipulator_action_msg()
                self.publishers[C.MANIPULATOR_ACTION_TOPIC].publish(
                    confirmation)
                return C.ACTION_STATE_IDLE

        return self.current_state

    def update_controller_states(self, new_state: str) -> None:
        """
        Change the robot's current state and broadcast a message to the subnodes.
        """
        self.current_state = new_state
        action_state_msg = self.create_action_state_msg()
        self.publishers[C.ACTION_STATE_TOPIC].publish(action_state_msg)

    def run(self) -> None:
        """
        Listen for incoming messages.
        """
        rospy.spin()


if __name__ == "__main__":
    controller = ActionController()
    controller.run()
