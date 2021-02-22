#!/usr/bin/env python3

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image


IMG_RAW_TOPIC = "camera/rgb/image_raw"
VISION_UPDATES_TOPIC = "vision_updates"

STATE_IDLE = 0
STATE_COLOR_SEARCH = 1
STATE_NUMBER_SEARCH = 2

COLOR_NONE = 0
COLOR_RED = 1
COLOR_GREEN = 2
COLOR_BLUE = 3
NUMBER_NONE = 0
NUMBER_ONE = 1
NUMBER_TWO = 2
NUMBER_THREE = 3

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


def mask_hue(img, hue):
    hue_lower_bound = hue - (HUE_RANGE / 2)
    hue_upper_bound = hue + (HUE_RANGE / 2)

    if hue_lower_bound < 0 or hue_upper_bound > 180:
        lower_bound_1 = np.array([0, MIN_SAT, MIN_VAL])
        upper_bound_1 = np.array([
            hue_upper_bound % 180, MAX_SAT, MAX_VAL])
        mask1 = cv2.inRange(img, lower_bound_1, upper_bound_1)

        lower_bound_2 = np.array([
            hue_lower_bound % 180, MIN_SAT, MIN_VAL])
        upper_bound_2 = np.array([180, MAX_SAT, MAX_VAL])
        mask2 = cv2.inRange(img, lower_bound_2, upper_bound_2)

        return mask0 + mask1
    else:
        lower_bound = np.array([hue_lower_bound, MIN_SAT, MIN_VAL])
        upper_bound = np.array([hue_upper_bound, MAX_SAT, MAX_VAL])
        mask = cv2.inRange(img, lower_bound, upper_bound)

        return mask


class VisionController(object):
    
    def __init__(self):
        rospy.init_node('q_bot_vision')

        self.pubs = initialize_publishers()
        initialize_subscribers()
        
        self.state = STATE_IDLE
        self.img_raw_counter = 0
        self.bridge = cv_bridge.CvBridge()

        
    def initialize_publishers(self):
        publishers = {}

        publishers[VISION_UPDATES_TOPIC] = rospy.Publisher(
            VISION_UPDATES_TOPIC, int, queue_size=QUEUE_SIZE
        )

        return publishers


    def initialize_subscribers(self):
        rospy.Subscriber(IMG_RAW_TOPIC, Image, self.update_state)


    def set_state(self, new_state, new_search_state):
        self.state = new_state
        self.search_state = new_search_state


    def process_image(self, img):
        self.img_raw_counter = (self.img_raw_counter + 1) % UPDATE_RATE

        if self.img_raw_counter == 0:
            if self.state == STATE_COLOR_SEARCH:
                mask_hue = 

            elif self.state == STATE_NUMBER_SEARCH:


    



            

    def image_to_ndarray(self, img):
        image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)




    def run(self):
        rospy.spin()