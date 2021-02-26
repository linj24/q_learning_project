#!/usr/bin/env python3
import numpy as np

ACTION_STATE_TOPIC = "action_state"
ROBOT_ACTION_TOPIC = "q_learning/RobotMoveDBToBlock"
QUEUE_SIZE = 10

ACTION_STATE_IDLE = "action_state_idle"
ACTION_STATE_SCOUT = "action_state_scout"
ACTION_STATE_MOVE_CENTER = "action_state_move_center"
ACTION_STATE_LOCATE_DUMBBELL = "action_state_locate_dumbbell"
ACTION_STATE_MOVE_DUMBBELL = "action_state_move_dumbbell"
ACTION_STATE_GRAB = "action_state_grab"
ACTION_STATE_LOCATE_BLOCK = "action_state_locate_block"
ACTION_STATE_MOVE_BLOCK = "action_state_move_block"
ACTION_STATE_RELEASE = "action_state_release"


MOVEMENT_STATE_IDLE = "movement_state_idle"
MOVEMENT_STATE_GO_TO_POSITION = "movement_state_go_to_position"
MOVEMENT_STATE_FIND_OBJECT = "movement_state_find_object"
MOVEMENT_STATE_FOLLOW_OBJECT = "movement_state_track_object"


VISION_STATE_IDLE = 0
VISION_STATE_COLOR_SEARCH = 1
VISION_STATE_NUMBER_SEARCH = 2

COLOR_NONE = "color_none"
COLOR_RED = "color_red"
COLOR_GREEN = "color_green"
COLOR_BLUE = "color_blue"
NUMBER_NONE = "number_none"
NUMBER_ONE = "number_one"
NUMBER_TWO = "number_two"
NUMBER_THREE = "number_three"


CMD_VEL_TOPIC = "cmd_vel"
SCAN_TOPIC = "scan"
ODOM_TOPIC = "odom"
IMG_RAW_TOPIC = "camera/rgb/image_raw"
IMG_CEN_TOPIC = "image_centroid"
QUEUE_SIZE = 10


MVMT_THRESH_LIN = 0.2
MVMT_THRESH_ANG = 0.6


# Proportional coefficient for linear velocity
KP_LIN = 0.3
# Proportional coefficient for angular velocity
KP_ANG = 0.01
# Angular velocity for turning corners
TURN_VEL = 200
# Safe distance from object
THRESHOLD = 0.7
# Greatest distance to start following the object at
DETECTION_LIMIT = 3.5

UPDATE_RATE = 1

LOWER_RED_1 = np.array([0, 50, 50])
UPPER_RED_1 = np.array([15, 255, 255])
LOWER_RED_2 = np.array([165, 50, 50])
UPPER_RED_2 = np.array([180, 255, 255])

RED_HUE = 0
GREEN_HUE = 60
BLUE_HUE = 120

HUE_RANGE = 30
MIN_SAT = 50
MAX_SAT = 255
MIN_VAL = 50
MAX_VAL = 255

LOWER_GREEN = np.array([45, 50, 50])
UPPER_GREEN = np.array([75, 255, 255])

LOWER_BLUE = np.array([105, 50, 50])
UPPER_BLUE = np.array([135, 255, 255])