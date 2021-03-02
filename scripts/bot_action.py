#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from q_learning_project.msg import RobotMoveDBToBlock, ActionState, ImgCen, ArmRaised, ManipulatorAction
from utils import find_distance
import constants as C


class ActionController(object):

    def __init__(self):
        rospy.init_node('q_bot_action')

        self.current_state = C.ACTION_STATE_IDLE
        self.starting_pose = Pose()
        self.current_robot_action = RobotMoveDBToBlock()
        self.initialized = False
        self.conditions = {
            "IN_CENTER": True,
            "IN_FRONT_OF_OBJECT": False,
            "HOLDING_DUMBBELL": False,
            "FACING_DUMBBELL": False,
            "FACING_BLOCK": False,
        }

        self.publishers = self.initialize_publishers()
        self.initialize_subscribers()

    def initialize_publishers(self) -> dict:
        publishers = {}
        publishers[C.ACTION_STATE_TOPIC] = rospy.Publisher(
            C.ACTION_STATE_TOPIC, ActionState, queue_size=C.QUEUE_SIZE
        )
        publishers[C.MANIPULATOR_ACTION_TOPIC] = rospy.Publisher(
            C.MANIPULATOR_ACTION_TOPIC, ManipulatorAction, queue_size=C.QUEUE_SIZE
        )
        return publishers

    def initialize_subscribers(self) -> None:
        rospy.Subscriber(C.ODOM_TOPIC, Odometry, self.process_odom)
        rospy.Subscriber(C.SCAN_TOPIC, LaserScan, self.process_scan)
        rospy.Subscriber(C.IMG_CEN_TOPIC, ImgCen, self.process_img_cen)
        rospy.Subscriber(C.MANIPULATOR_ACTION_TOPIC,
                         ManipulatorAction, self.process_manipulator_action)
        rospy.Subscriber(C.ARM_RAISED_TOPIC, ArmRaised,
                         self.process_arm_raised)

    def create_ActionState_msg(self) -> ActionState:
        action_state = ActionState()
        action_state.action_state = self.current_state
        action_state.robot_db = self.current_robot_action.robot_db
        action_state.block_id = self.current_robot_action.block_id
        return action_state

    def create_ManipulatorAction_msg(self) -> ManipulatorAction:
        confirmation = ManipulatorAction()
        confirmation.is_confirmation = True
        confirmation.block_id = self.current_robot_action.block_id
        confirmation.robot_db = self.current_robot_action.robot_db
        return confirmation

    def process_odom(self, odom_data: Odometry) -> None:
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
        front_distance = min(
            scan_data.ranges[:C.FRONT_ANGLE_RANGE//2], scan_data.ranges[-C.FRONT_ANGLE_RANGE//2:])
        if front_distance < C.SAFE_DISTANCE:
            self.conditions["IN_FRONT_OF_OBJECT"] = True
        else:
            self.conditions["IN_FRONT_OF_OBJECT"] = False
        self.update_controller_states(self.get_next_state())

    def process_img_cen(self, img_cen_data: ImgCen) -> None:
        if (img_cen_data.target != C.TARGET_NONE and
                abs(img_cen_data.center_x) < C.IMG_CEN_PIXEL_THRESHOLD):
            if img_cen_data.vision_state == C.VISION_STATE_COLOR_SEARCH:
                self.conditions["FACING_DUMBBELL"] = True
                self.conditions["FACING_BLOCK"] = False
            elif img_cen_data.vision_state == C.VISION_STATE_NUMBER_SEARCH:
                self.conditions["FACING_DUMBBELL"] = False
                self.conditions["FACING_BLOCK"] = True
        else:
            self.conditions["FACING_DUMBBELL"] = False
            self.conditions["FACING_BLOCK"] = False
        self.update_controller_states(self.get_next_state())

    def process_manipulator_action(self, action: ManipulatorAction) -> None:
        if not action.is_confirmation:
            self.current_robot_action.block_id = action.block_id
            self.current_robot_action.robot_db = action.robot_db
            self.update_controller_states(C.ACTION_STATE_MOVE_CENTER)

    def process_arm_raised(self, raised_flag: ArmRaised) -> None:
        self.conditions["HOLDING_DUMBBELL"] = raised_flag.arm_raised

    def get_next_state(self) -> str:
        if self.current_state == C.ACTION_STATE_MOVE_CENTER:
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
            if (self.conditions["IN_FRONT_OF_OBJECT"] and
                    self.conditions["FACING_DUMBBELL"]):
                return C.ACTION_STATE_GRAB

            elif not self.conditions["FACING_DUMBBELL"]:
                return C.ACTION_STATE_LOCATE_DUMBBELL

        elif self.current_state == C.ACTION_STATE_GRAB:
            if self.conditions["HOLDING_DUMBBELL"]:
                return C.ACTION_STATE_MOVE_CENTER

        elif self.current_state == C.ACTION_STATE_LOCATE_BLOCK:
            if self.conditions["FACING_BLOCK"]:
                return C.ACTION_STATE_MOVE_BLOCK

        elif self.current_state == C.ACTION_STATE_MOVE_BLOCK:
            if (self.conditions["IN_FRONT_OF_OBJECT"] and
                    self.conditions["FACING_BLOCK"]):
                return C.ACTION_STATE_RELEASE

            elif not self.conditions["FACING_BLOCK"]:
                return C.ACTION_STATE_LOCATE_BLOCK

        elif self.current_state == C.ACTION_STATE_RELEASE:
            if not self.conditions["HOLDING_DUMBBELL"]:
                confirmation = self.create_ManipulatorAction_msg()
                self.publishers[C.MANIPULATOR_ACTION_TOPIC].publish(
                    confirmation)
                return C.ACTION_STATE_IDLE

        return self.current_state

    def update_controller_states(self, new_state: str) -> None:
        self.current_state = new_state
        action_state_msg = self.create_ActionState_msg()
        self.publishers[C.ACTION_STATE_TOPIC].publish(action_state_msg)

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    controller = ActionController()
    controller.run()
