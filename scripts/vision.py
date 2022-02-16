#!/usr/bin/env python3

"""
Controller object handling all image processing based on the robot state.
"""

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
import constants as C
from utils import wrap_bounds
from q_learning_project.msg import ActionState, ImgCen


def mask_hue(img, hue):
    """
    Create a mask for an image that only includes values in a certain hue range.
    """

    # Deal with the red mask, which wraps around 180
    hue_lower_bound, hue_upper_bound = wrap_bounds(hue, 180, C.HUE_RANGE)

    if hue_lower_bound > hue_upper_bound:
        # If the color being masked is red, do it in two parts
        lower_bound_1 = np.array([0, C.MIN_SAT, C.MIN_VAL]).astype(int)
        upper_bound_1 = np.array([
            hue_upper_bound, C.MAX_SAT, C.MAX_VAL]).astype(int)
        mask1 = cv2.inRange(img, lower_bound_1, upper_bound_1)

        lower_bound_2 = np.array([
            hue_lower_bound, C.MIN_SAT, C.MIN_VAL]).astype(int)
        upper_bound_2 = np.array(
            [180, C.MAX_SAT, C.MAX_VAL]).astype(int)
        mask2 = cv2.inRange(img, lower_bound_2, upper_bound_2)

        return mask1 + mask2

    else:
        # If the color being masked is green or blue, do it in one part
        lower_bound = np.array(
            [hue_lower_bound, C.MIN_SAT, C.MIN_VAL]).astype(int)
        upper_bound = np.array(
            [hue_upper_bound, C.MAX_SAT, C.MAX_VAL]).astype(int)
        mask = cv2.inRange(img, lower_bound, upper_bound)

        return mask


def calc_color_centroid(img, mask):
    """
    Calculate the centroid of a masked image.
    """
    # Code adapted from class_meeting_03
    height, width, _ = img.shape
    moments = cv2.moments(mask)

    if moments['m00'] > 0:
        cen_x = int(moments['m10']/moments['m00'])
        cen_y = int(moments['m01']/moments['m00'])
        return (cen_x - (width//2), cen_y - (height//2))

    return None


def calc_tag_center(img, corners):
    """
    Calculate the center of a tag from its coordinates.
    """
    height, width = img.shape
    center = np.sum(corners[0], axis=0) // 4
    return (center[0] - (width//2), center[1] - (height//2))


class VisionController():
    """
    Tracks the robot's current state and processes raw images.
    """

    def __init__(self):
        rospy.init_node('q_bot_vision')

        self.current_state = C.VISION_STATE_IDLE
        self.color_search_target = C.TARGET_NONE
        self.number_search_target = C.TARGET_NONE
        self.bridge = cv_bridge.CvBridge()

        self.publishers = self.initialize_publishers()
        self.initialize_subscribers()

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

    def initialize_publishers(self):
        """
        Initialize all the publishers this node will use.
        """
        publishers = {}

        publishers[C.IMG_CEN_TOPIC] = rospy.Publisher(
            C.IMG_CEN_TOPIC, ImgCen, queue_size=C.QUEUE_SIZE
        )

        return publishers

    def initialize_subscribers(self):
        """
        Initialize all the subscribers this node will use.
        """
        rospy.Subscriber(C.IMG_RAW_TOPIC, Image, self.process_image)
        rospy.Subscriber(
            C.ACTION_STATE_TOPIC,
            ActionState,
            self.process_action_state)

    def set_state(self, new_state):
        """
        Set the current vision state.
        """
        if new_state != self.current_state:
            rospy.loginfo(f"[Vision] Current state is {new_state}")
        self.current_state = new_state

    def set_color_search_target(self, search_target):
        """
        Set the current color search target.
        """
        self.color_search_target = search_target

    def set_number_search_target(self, search_target):
        """
        Set the current number search target.
        """
        self.number_search_target = search_target

    def create_img_cen_msg(self, cv2_img):
        """
        Create an ImgCen message from an image based on the current vision state.
        """
        img_cen_msg = ImgCen()
        img_cen_msg.vision_state = self.current_state
        center = None

        if self.current_state == C.VISION_STATE_COLOR_SEARCH:
            # If currently searching for a color, calculate a color centroid
            img_cen_msg.target = self.color_search_target

            hsv_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
            hue = C.COLOR_HUE_MAP[self.color_search_target]

            mask = mask_hue(hsv_img, hue)
            center = calc_color_centroid(hsv_img, mask)

        elif self.current_state == C.VISION_STATE_NUMBER_SEARCH:
            # If currently searching for a number, calculate a box centroid from
            # keras-ocr recognitions
            img_cen_msg.target = self.number_search_target
            
            gray_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected_points = cv2.aruco.detectMarkers(gray_img, self.aruco_dict)

            if ids is not None:
                for i, id in enumerate(ids.flatten()):
                    # Check if the target number was one of the detected tags
                    if str(id) == self.number_search_target:
                        center = calc_tag_center(gray_img, corners[i])

        if center is None:
            # Say that the desired object wasn't detected if it wasn't found
            img_cen_msg.target = C.TARGET_NONE
            img_cen_msg.center_x = 0.0
            img_cen_msg.center_y = 0.0
        else:
            img_cen_msg.center_x = center[0]
            img_cen_msg.center_y = center[1]

        return img_cen_msg

    def process_action_state(self, action_state):
        """
        Receive an action state and update the vision state accordingly.
        """
        self.set_color_search_target(action_state.robot_db)
        self.set_number_search_target(str(action_state.block_id))
        new_state = action_state.action_state

        # Process the camera feed when the bot is facing an object and not moving;
        # if searching for a dumbbell, detect a color
        # if searching for a block, detect a number
        if new_state == C.ACTION_STATE_IDLE:
            self.set_state(C.VISION_STATE_IDLE)

        elif new_state == C.ACTION_STATE_MOVE_CENTER:
            self.set_state(C.VISION_STATE_IDLE)

        elif new_state == C.ACTION_STATE_LOCATE_DUMBBELL:
            self.set_state(C.VISION_STATE_COLOR_SEARCH)

        elif new_state == C.ACTION_STATE_CENTER_DUMBBELL:
            self.set_state(C.VISION_STATE_COLOR_SEARCH)

        elif new_state == C.ACTION_STATE_MOVE_DUMBBELL:
            self.set_state(C.VISION_STATE_COLOR_SEARCH)

        elif new_state == C.ACTION_STATE_GRAB:
            self.set_state(C.VISION_STATE_IDLE)

        elif new_state == C.ACTION_STATE_LOCATE_BLOCK:
            self.set_state(C.VISION_STATE_NUMBER_SEARCH)

        elif new_state == C.ACTION_STATE_CENTER_BLOCK:
            self.set_state(C.VISION_STATE_NUMBER_SEARCH)

        elif new_state == C.ACTION_STATE_MOVE_BLOCK:
            self.set_state(C.VISION_STATE_NUMBER_SEARCH)

        elif new_state == C.ACTION_STATE_RELEASE:
            self.set_state(C.VISION_STATE_IDLE)

        elif new_state == C.ACTION_STATE_BACK_AWAY:
            self.set_state(C.VISION_STATE_IDLE)

    def process_image(self, img):
        """
        Receive an image, calculate an object's location in pixels,
        and notify the action controller.
        """
        # Process an image if asked to
        cv2_img = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        img_cen_msg = self.create_img_cen_msg(cv2_img)
        self.publishers[C.IMG_CEN_TOPIC].publish(img_cen_msg)

    def run(self):
        """
        Listen for incoming messages.
        """
        rospy.spin()


if __name__ == "__main__":
    controller = VisionController()
    controller.run()
