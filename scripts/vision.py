#!/usr/bin/env python3

import rospy, cv2, cv_bridge, keras_ocr
import numpy as np
from sensor_msgs.msg import Image
from utils import wrap_bounds
from q_learning_project.msg import ImgCen
import constants as C


def mask_hue(img, hue):
    hue_lower_bound, hue_upper_bound = wrap_bounds(hue, 180, C.HUE_RANGE)

    if hue_lower_bound > hue_upper_bound:
        lower_bound_1 = np.array([0, C.MIN_SAT, C.MIN_VAL])
        upper_bound_1 = np.array([
            hue_upper_bound, C.MAX_SAT, C.MAX_VAL])
        mask1 = cv2.inRange(img, lower_bound_1, upper_bound_1)

        lower_bound_2 = np.array([
            hue_lower_bound, C.MIN_SAT, C.MIN_VAL])
        upper_bound_2 = np.array([180, C.MAX_SAT, C.MAX_VAL])
        mask2 = cv2.inRange(img, lower_bound_2, upper_bound_2)

        return mask1 + mask2
    else:
        lower_bound = np.array([hue_lower_bound, C.MIN_SAT, C.MIN_VAL])
        upper_bound = np.array([hue_upper_bound, C.MAX_SAT, C.MAX_VAL])
        mask = cv2.inRange(img, lower_bound, upper_bound)

        return mask


def calc_color_centroid(img, mask):
    # Code from class meeting 03
    h, w, _ = img.shape
    search_top = int(3*h/4)
    search_bot = int(3*h/4 + 20)
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0

    M = cv2.moments(mask)

    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return (cx - (h//2), cy - (w//2))

    return None


def calc_box_center(box):
    box_center_x = 0
    box_center_y = 0
    for box_corner_y, box_corner_x in box:
        box_center_x = box_center_x + box_corner_x
        box_center_y = box_center_y + box_corner_y
    box_center = (box_center_x / 4, box_center_y / 4)

    return box_center


class VisionController(object):
    
    def __init__(self):
        #rospy.init_node('q_bot_vision')

        self.publishers = self.initialize_publishers()
        self.initialize_subscribers()
        
        self.current_state = C.VISION_STATE_IDLE
        self.search_target = C.TARGET_NONE
        self.img_raw_counter = 0
        self.bridge = cv_bridge.CvBridge()
        self.pipeline = keras_ocr.pipeline.Pipeline()

        
    def initialize_publishers(self):
        publishers = {}

        publishers[C.IMG_CEN_TOPIC] = rospy.Publisher(
            C.IMG_CEN_TOPIC, ImgCen, queue_size=C.QUEUE_SIZE
        )

        return publishers


    def initialize_subscribers(self):
        rospy.Subscriber(C.IMG_RAW_TOPIC, Image, self.process_image)


    def set_state(self, new_state):
        self.current_state = new_state


    def set_search_target(self, search_target):
        self.search_target = search_target
        

    def color_state_to_hue(self):
        if self.search_target == C.COLOR_RED:
            return C.RED_HUE
        elif self.search_target == C.COLOR_GREEN:
            return C.GREEN_HUE
        elif self.search_target == C.COLOR_BLUE:
            return C.BLUE_HUE
        else:
            return


    def create_ImgCen_msg(self, csv_img):
        img_cen_msg = ImgCen()
        img_cen_msg.vision_state = self.current_state
        center = None

        if self.current_state == C.VISION_STATE_COLOR_SEARCH:
            hsv_img = cv2.cvtColor(csv_img, cv2.COLOR_BGR2HSV)
            hue = self.color_state_to_hue()

            mask = mask_hue(hsv_img, hue)
            center = calc_color_centroid(hsv_img, mask)

        elif self.current_state == C.VISION_STATE_NUMBER_SEARCH:
            prediction_group = self.pipeline.recognize([csv_img])[0]

            for word, box in prediction_group:
                if word == self.number_search_target:
                    box_center = calc_box_center(box)
        
        if center is None:
            img_cen_msg.target = C.TARGET_NONE
            img_cen_msg.center_x = 0.0
            img_cen_msg.center_y = 0.0
        else:
            img_cen_msg.target = self.search_target
            img_cen_msg.center_x = center[0]
            img_cen_msg.center_y = center[1]
        
        return img_cen_msg


    def process_image(self, img):
        self.img_raw_counter = (self.img_raw_counter + 1) % C.IMG_RAW_UPDATE_RATE

        if self.img_raw_counter == 0:
            if self.current_state != C.VISION_STATE_IDLE:
                csv_img = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
                img_cen_msg = self.create_ImgCen_msg(csv_img)
                self.publishers[C.IMG_CEN_TOPIC].publish(img_cen_msg)
                        

    def run(self):
        rospy.spin()