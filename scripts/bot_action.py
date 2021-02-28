#!/usr/bin/env python3


import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from arm_manipulation import ArmController
from movement import MovementController
from vision import VisionController
from utils import find_distance
from q_learning_project.msg import RobotMoveDBToBlock, ImgCen
import constants as C


class ActionController(object):
    
    def __init__(self):
        #rospy.init_node('q_bot_action')

        self.arm_controller = ArmController()
        self.movement_controller = MovementController()
        self.vision_controller = VisionController()

        self.initialize_subscribers()

        self.current_state = C.ACTION_STATE_IDLE
        self.search_target = C.TARGET_NONE
        self.conditions = {
            "IN_CENTER": True,
            "IN_FRONT_OF_OBJECT": False,
            "HOLDING_DUMBBELL": False,
            "FACING_DUMBBELL": False,
            "FACING_BLOCK": False,
        }


    def initialize_subscribers(self) -> None:
        rospy.Subscriber(C.ODOM_TOPIC, Odometry, self.process_odom)
        rospy.Subscriber(C.SCAN_TOPIC, LaserScan, self.process_scan)
        rospy.Subscriber(C.IMG_CEN_TOPIC, ImgCen, self.process_img_cen)
        rospy.Subscriber(C.ROBOT_ACTION_TOPIC, RobotMoveDBToBlock, self.process_action)


    def process_odom(self, odom_data: Odometry) -> None:
        distance = find_distance(
            self.movement_controller.starting_pose, odom_data.pose.pose)
        if distance < C.CENTER_RADIUS:
            self.conditions["IN_CENTER"] = True
        else:
            self.conditions["IN_CENTER"] = False
        self.update_controller_states(C.ACTION_STATE_MOVE_CENTER)


    def process_scan(self, scan_data: LaserScan) -> None:
        front_distance = scan_data.ranges[0]
        if front_distance < C.SAFE_DISTANCE:
            self.conditions["IN_FRONT_OF_OBJECT"] = True
        else:
            self.conditions["IN_FRONT_OF_OBJECT"] = False
        self.update_controller_states(C.ACTION_STATE_MOVE_CENTER)


    def process_img_cen(self, img_cen_data: ImgCen) -> None:
        if abs(img_cen_data.center_x) < C.IMG_CEN_PIXEL_THRESHOLD:
            if self.vision_controller.set_state(C.VISION_STATE_COLOR_SEARCH):
                self.conditions["FACING_DUMBBELL"] = True
                self.conditions["FACING_BLOCK"] = False
            elif self.vision_controller.set_state(C.VISION_STATE_NUMBER_SEARCH):
                self.conditions["FACING_DUMBBELL"] = False
                self.conditions["FACING_BLOCK"] = True
        else:
            self.conditions["FACING_DUMBBELL"] = False
            self.conditions["FACING_BLOCK"] = False
        self.update_controller_states(C.ACTION_STATE_MOVE_CENTER)
        

    def process_action(self, action: RobotMoveDBToBlock) -> None:
        self.vision_controller.set_color_search_target(action.robot_db)
        self.vision_controller.set_number_search_target(action.block_id)
        self.update_controller_states(C.ACTION_STATE_MOVE_CENTER)


    def get_next_state(self) -> str:
        if self.current_state == C.ACTION_STATE_IDLE:
            return C.ACTION_STATE_MOVE_CENTER

        elif self.current_state == C.ACTION_STATE_MOVE_CENTER:
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
                return C.ACTION_STATE_IDLE

        return self.current_state


    def update_controller_states(self, new_state: str) -> None:
        self.current_state = new_state
        if new_state == C.ACTION_STATE_IDLE:
            self.arm_controller.set_state(C.ARM_STATE_IDLE)
            self.movement_controller.set_state(C.MOVEMENT_STATE_IDLE)
            self.vision_controller.set_state(C.VISION_STATE_IDLE)

        elif new_state == C.ACTION_STATE_MOVE_CENTER:
            self.arm_controller.set_state(C.ARM_STATE_IDLE)
            self.movement_controller.set_state(C.MOVEMENT_STATE_GO_TO_POSITION)
            self.vision_controller.set_state(C.VISION_STATE_IDLE)

        elif new_state == C.ACTION_STATE_LOCATE_DUMBBELL:
            self.arm_controller.set_state(C.ARM_STATE_DOWN)
            self.movement_controller.set_state(C.MOVEMENT_STATE_FIND_OBJECT)
            self.vision_controller.set_state(C.VISION_STATE_COLOR_SEARCH)

        elif new_state == C.ACTION_STATE_MOVE_DUMBBELL:
            self.arm_controller.set_state(C.ARM_STATE_DOWN)
            self.movement_controller.set_state(C.MOVEMENT_STATE_FOLLOW_OBJECT)
            self.vision_controller.set_state(C.VISION_STATE_NUMBER_SEARCH)

        elif new_state == C.ACTION_STATE_GRAB:
            self.arm_controller.set_state(C.ARM_STATE_GRABBING)
            self.movement_controller.set_state(C.MOVEMENT_STATE_IDLE)
            self.vision_controller.set_state(C.VISION_STATE_IDLE)

        elif new_state == C.ACTION_STATE_LOCATE_BLOCK:
            self.arm_controller.set_state(C.ARM_STATE_UP)
            self.movement_controller.set_state(C.MOVEMENT_STATE_FIND_OBJECT)
            self.vision_controller.set_state(C.VISION_STATE_NUMBER_SEARCH)

        elif new_state == C.ACTION_STATE_MOVE_BLOCK:
            self.arm_controller.set_state(C.ARM_STATE_UP)
            self.movement_controller.set_state(C.MOVEMENT_STATE_FOLLOW_OBJECT)
            self.vision_controller.set_state(C.VISION_STATE_NUMBER_SEARCH)

        elif new_state == C.ACTION_STATE_RELEASE:
            self.arm_controller.set_state(C.ARM_STATE_RELEASING)
            self.movement_controller.set_state(C.MOVEMENT_STATE_IDLE)
            self.vision_controller.set_state(C.VISION_STATE_IDLE)


    def run(self) -> None:
        self.movement_controller.run()
        self.vision_controller.run()
        rospy.spin()