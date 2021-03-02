#!/usr/bin/env python3

import rospy
from bot_action import ActionController
from q_learning_project.msg import RobotMoveDBToBlock, QMatrix
import constants as C


class Robot(object):
    
    def __init__(self):
        rospy.init_node('q_learning')

        self.action_controller = ActionController()

        self.publishers = self.initialize_publishers()
        self.initialize_subscribers()


    def initialize_publishers(self) -> dict:
        publishers = {}

        publishers[C.ROBOT_ACTION_TOPIC] = rospy.Publisher(
            C.ROBOT_ACTION_TOPIC, RobotMoveDBToBlock, queue_size=C.QUEUE_SIZE
        )

        return publishers


    def initialize_subscribers(self) -> None:
        rospy.Subscriber(C.Q_MATRIX_TOPIC, QMatrix, self.process_q_matrix)


    def process_q_matrix(self, q_matrix: QMatrix) -> None:
        pass

        
    def run(self) -> None:
        self.action_controller.run()
        rospy.spin()


if __name__=="__main__":
    bot = Robot()
    bot.run()