#!/usr/bin/env python3

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from utils import ImgCentroidMsg
import constants as C


def mask_hue(img, hue):
    hue_lower_bound = hue - (C.HUE_RANGE / 2)
    hue_upper_bound = hue + (C.HUE_RANGE / 2)

    if hue_lower_bound < 0 or hue_upper_bound > 180:
        lower_bound_1 = np.array([0, C.MIN_SAT, C.MIN_VAL])
        upper_bound_1 = np.array([
            hue_upper_bound % 180, C.MAX_SAT, C.MAX_VAL])
        mask1 = cv2.inRange(img, lower_bound_1, upper_bound_1)

        lower_bound_2 = np.array([
            hue_lower_bound % 180, C.MIN_SAT, C.MIN_VAL])
        upper_bound_2 = np.array([180, C.MAX_SAT, C.MAX_VAL])
        mask2 = cv2.inRange(img, lower_bound_2, upper_bound_2)

        return mask1 + mask2
    else:
        lower_bound = np.array([hue_lower_bound, C.MIN_SAT, C.MIN_VAL])
        upper_bound = np.array([hue_upper_bound, C.MAX_SAT, C.MAX_VAL])
        mask = cv2.inRange(img, lower_bound, upper_bound)

        return mask


def calc_centroid(img, mask):
    # Code from class meeting 03
    h, w, d = img.shape
    search_top = int(3*h/4)
    search_bot = int(3*h/4 + 20)
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0

    M = cv2.moments(mask)

    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return (cx, cy)

    return (None, None)


class VisionController(object):
    
    def __init__(self):
        rospy.init_node('q_bot_vision')

        self.pubs = self.initialize_publishers()
        self.initialize_subscribers()
        
        self.state = C.VISION_STATE_IDLE
        self.color_search_target = C.COLOR_NONE
        self.number_search_target = C.NUMBER_NONE
        self.img_raw_counter = 0
        self.bridge = cv_bridge.CvBridge()

        
    def initialize_publishers(self):
        publishers = {}

        publishers[C.IMG_CEN_TOPIC] = rospy.Publisher(
            C.IMG_CEN_TOPIC, ImgCentroidMsg, queue_size=C.QUEUE_SIZE
        )

        return publishers


    def initialize_subscribers(self):
        rospy.Subscriber(C.IMG_RAW_TOPIC, Image, self.process_image)


    def set_state(self, new_state, new_search_state):
        self.state = new_state
        self.search_state = new_search_state


    def color_state_to_hue(self):
        if self.color_search_target == C.COLOR_RED:
            return C.RED_HUE
        elif self.color_search_target == C.COLOR_GREEN:
            return C.GREEN_HUE
        elif self.color_search_target == C.COLOR_BLUE:
            return C.BLUE_HUE
        else:
            return


    def process_image(self, img):
        self.img_raw_counter = (self.img_raw_counter + 1) % C.UPDATE_RATE

        if self.img_raw_counter == 0:
            if self.state == C.VISION_STATE_COLOR_SEARCH:

                hsv_img = self.image_to_ndarray(img)
                hue = self.color_state_to_hue()

                mask = mask_hue(hsv_img, hue)
                centroid = calc_centroid(hsv_img, mask)
                
                self.pubs[C.IMG_CEN_TOPIC].publish(ImgCentroidMsg(hue, centroid))

            elif self.state == C.VISION_STATE_NUMBER_SEARCH:
                pass


    def image_to_ndarray(self, img):
        image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return hsv


    def run(self):
        rospy.spin()