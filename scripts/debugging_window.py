import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from q_learning_project.msg import ManipulatorAction
import vision as v
import constants as C

class Debugger():
    def __init__(self):

        cv2.namedWindow("window", 1)

        self.bridge = cv_bridge.CvBridge()

        self.sub_img = rospy.Subscriber(C.IMG_RAW_TOPIC, Image, self.process_image)
        self.sub_act = rospy.Subscriber(C.MANIPULATOR_ACTION_TOPIC, ManipulatorAction,
                                        self.process_manipulator_action)
        
        self.target_color = None
        self.target_tag = None

        self.running = True

        self.display_image = None
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)


    def process_manipulator_action(self, msg):                                        
        self.target_color = msg.robot_db
        self.target_tag = msg.block_id

    def process_image(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.target_color is not None:
            hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            hue = C.COLOR_HUE_MAP[self.target_color]

            mask = v.mask_hue(hsv_img, hue)
            center = v.calc_color_centroid(hsv_img, mask)

            display_center = (center[0] + (image.shape[1]//2), center[1] + (image.shape[0]//2))

            cv2.circle(image, display_center, 20, (0,0,255), -1)

        if self.target_tag is not None:
            corners, ids, _ = cv2.aruco.detectMarkers(image, self.aruco_dict)
            cv2.aruco.drawDetectedMarkers(image, corners, ids)

        self.display_image = image

    def run(self):
        while self.running:
            if self.display_image is not None:
                cv2.imshow("window", self.display_image)

            if cv2.waitKey(3) == ord('q'):
                self.running = False



if __name__ == "__main__":
    rospy.init_node("debugger")
    dbg = Debugger()
    dbg.run()
