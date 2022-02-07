#!/usr/bin/env python3

"""
A list of constants used throughout various modules.
"""

import numpy as np

MANIPULATOR_ACTION_TOPIC = "/q_learning/manipulator_action"
CMD_VEL_TOPIC = "cmd_vel"
SCAN_TOPIC = "scan"
ODOM_TOPIC = "odom"
IMG_RAW_TOPIC = "camera/rgb/image_raw"
IMG_CEN_TOPIC = "q_learning/img_cen"
ROBOT_ACTION_TOPIC = "q_learning/robot_action"
Q_MATRIX_TOPIC = "q_learning/q_matrix"
REWARD_TOPIC = "q_learning/reward"
ACTION_STATE_TOPIC = "q_learning/states/action"
ARM_RAISED_TOPIC = "q_learning/arm_raised"
QUEUE_SIZE = 10

CENTER_RADIUS = 0.2

CONTROLLER_STATE_INIT = "controller_state_init"
CONTROLLER_STATE_LEARN = "controller_state_learn"
CONTROLLER_STATE_LOOP = "controller_state_loop"

ACTION_STATE_IDLE = "action_state_idle"
ACTION_STATE_MOVE_CENTER = "action_state_move_center"
ACTION_STATE_LOCATE_DUMBBELL = "action_state_locate_dumbbell"
ACTION_STATE_MOVE_DUMBBELL = "action_state_move_dumbbell"
ACTION_STATE_GRAB = "action_state_grab"
ACTION_STATE_LOCATE_BLOCK = "action_state_locate_block"
ACTION_STATE_MOVE_BLOCK = "action_state_move_block"
ACTION_STATE_RELEASE = "action_state_release"
ACTION_STATE_BACK_AWAY = "action_state_back_away"
ACTION_STATE_CENTER_BLOCK = "action_state_center_block"
ACTION_STATE_CENTER_DUMBBELL = "action_state_center_dumbbell"


ARM_STATE_IDLE = "arm_state_idle"
ARM_STATE_DOWN = "arm_state_down"
ARM_STATE_UP = "arm_state_up"
ARM_STATE_GRABBING = "arm_state_grabbing"
ARM_STATE_RELEASING = "arm_state_releasing"


MOVEMENT_STATE_IDLE = "movement_state_idle"
MOVEMENT_STATE_GO_TO_POSITION = "movement_state_go_to_position"
MOVEMENT_STATE_FIND_OBJECT = "movement_state_find_object"
MOVEMENT_STATE_CENTER_OBJECT = "movement_state_center_object"
MOVEMENT_STATE_FOLLOW_OBJECT = "movement_state_track_object"
MOVEMENT_STATE_APPROACH_OBJECT = "movement_state_approach_object"
MOVEMENT_STATE_BACK_AWAY = "movement_state_back_away"


VISION_STATE_IDLE = "vision_state_idle"
VISION_STATE_COLOR_SEARCH = "vision_state_color_search"
VISION_STATE_NUMBER_SEARCH = "vision_state_number_search"


TARGET_NONE = "none"
COLOR_RED = "red"
COLOR_GREEN = "green"
COLOR_BLUE = "blue"
NUMBER_ONE = 1
NUMBER_TWO = 2
NUMBER_THREE = 3


ARM_JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']
ARM_JOINT_GOAL_DOWN = [0.000, 0.025, 1.000, -1.000]
ARM_JOINT_GOAL_UP = [0.0000, -1.500, 1.000, -1.000]
# ARM_JOINT_GOAL_UP = [0.000, -1.800, 1.300, -1.200]
GRIPPER_JOINT_NAMES = ['gripper', 'gripper_sub']
GRIPPER_JOINT_GOAL_OPEN = [0.025, 0.025]
GRIPPER_JOINT_GOAL_CLOSED = [0.005, 0.005]


MVMT_THRESH_LIN = 0.2
MVMT_THRESH_ANG = 0.6


# Proportional coefficient for linear velocity
KP_LIN = 0.25
# Proportional coefficient for angular velocity
KP_ANG = 0.5
# Angular velocity for searching for objects
SAFE_DISTANCE_GRAB = 0.18
SAFE_DISTANCE_RELEASE = 0.35
BACK_AWAY_DISTANCE = 1
DETECTION_LIMIT = 3.5
LOCK_ON_RANGE = 24
FRONT_ANGLE_RANGE = 10
CENTER_ANGLE_RANGE = 5
APPROACH_SPEED = 0.005
BACK_AWAY_SPEED = -0.1
LOCK_ON_MODIFIER_FOLLOW = 0.03
LOCK_ON_MODIFIER_APPROACH = 0.1
OBJ_DIST_DIFF = 0.4
IMG_CEN_COLOR_PIXEL_THRESHOLD = 60
IMG_CEN_NUMBER_PIXEL_THRESHOLD = 80
IMG_CEN_ANGLE_MODIFIER = 0.001
SEARCH_TURN_VEL = 0.3
MIN_ANG_VEL = 0.1
TURN_LEFT = 1
TURN_RIGHT = -1
RIGHT_BLOCK_ANGLE_RANGE = range(120, 150)
LEFT_BLOCK_ANGLE_RANGE = range(210, 240)

LOWER_RED_1 = np.array([0, 50, 50])
UPPER_RED_1 = np.array([15, 255, 255])
LOWER_RED_2 = np.array([165, 50, 50])
UPPER_RED_2 = np.array([180, 255, 255])

RED_HUE = 0
GREEN_HUE = 60
BLUE_HUE = 120

COLOR_HUE_MAP = {
    COLOR_RED: RED_HUE,
    COLOR_GREEN: GREEN_HUE,
    COLOR_BLUE: BLUE_HUE
}

HUE_RANGE = 30
MIN_SAT = 150
MAX_SAT = 255
MIN_VAL = 150
MAX_VAL = 255

LOWER_GREEN = np.array([45, 50, 50])
UPPER_GREEN = np.array([75, 255, 255])

LOWER_BLUE = np.array([105, 50, 50])
UPPER_BLUE = np.array([135, 255, 255])
