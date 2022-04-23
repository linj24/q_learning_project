#!/usr/bin/env python3

import rospy

from std_msgs.msg import Header
from q_learning_project.msg import QLearningReward, QMatrix, QMatrixRow, RobotMoveDBToTag
import constants as C

import numpy as np
import os
import random



# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):

    def __init__(self):
        # Initialize this node
        rospy.init_node('q_learning')

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt", dtype=int)

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { dumbbell: "red", tag: 1}
        self.colors = ["red", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt", dtype=int)
        # self.actions = [{"dumbbell": colors[d], "tag": t} for d, t in self.actions]

        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the red, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at tag 1, and blue at tag 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the tag number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt", dtype=int)

        # Setup publisher for changes to QMatrix and when we want to send an action
        self.qmat_pub = rospy.Publisher("q_learning/q_matrix", QMatrix, queue_size=10)
        self.action_pub = rospy.Publisher("q_learning/robot_action", RobotMoveDBToTag, queue_size=10)
        # Setup subcriber for rewards
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.reward_received)

        # Sleep for one second to setup subscriber and publishers
        rospy.sleep(1.0)
        self.qmat = np.zeros((len(self.states), len(self.actions)), dtype=int)
        self.reward = None
    
    
    def run(self):
        # Converge the matrix
        self.do_qlearn()
        self.save_q_matrix()


    def get_rand_action(self, start_state):
        """
        Returns a (rand_action, final_state) uniformly sampled from the valid actions
        starting at start_state
        If there are no valid actions, return -1
        """
        # actions stores a list of actions indexed by end state
        actions = self.action_matrix[start_state]
        valid_actions = [action for action in actions if action != -1]
        if not valid_actions:
            return (-1, -1)
        action = random.choice(valid_actions)
        # get index in actions of the action
        final_state = np.where(actions == action)[0][0]
        return (action, final_state)
        
    def update_q(self, state, action, next_state, reward, alpha, gamma):
        """
        Updates self.qmat[curr_state][action] with the given reward,
        parameters, and next state. 
        Returns True if the q was updated, False otherwise. 
        
        """
        last_q = self.qmat[state][action]
        next_max_Q = np.max(self.qmat[next_state])
        updated_q = last_q + alpha * (reward + gamma * next_max_Q - last_q)
        self.qmat[state][action] = updated_q
        return updated_q != last_q


    def publish_qmat(self):
        """
        Converts self.qmat into a QMatrix() object and 
        publishes it to /q_learning/q_matrix
        """
        qmat_to_pub = QMatrix()
        qmat_to_pub.header = Header(stamp=rospy.Time.now())
        rows = []
        for row in self.qmat:
            qmat_row = QMatrixRow()
            qmat_row.q_matrix_row = list(row)
            rows.append(qmat_row)

        qmat_to_pub.q_matrix = rows
        self.qmat_pub.publish(qmat_to_pub)

    def save_q_matrix(self):
        np.savetxt(os.path.join(os.path.dirname(__file__), "q_matrix", "q_matrix.txt"), self.qmat)

    def do_qlearn(self):
        last_update_iter = curr_iter = 0
        # state 0 is everything at origin
        curr_state = 0
        converge_threshold = 300
        alpha = 1
        # Consider fine-tuning gamma choice
        gamma = 0.5
        # Converge after we have not updated qmat for converge_threshold iterations
        while curr_iter - last_update_iter < converge_threshold:
            (action, next_state) = self.get_rand_action(curr_state)
            if action == -1:
                # No valid actions left; all DBs at a block and this iteration is over
                rospy.sleep(C.Q_SLEEP_DURATION)
                curr_state = 0
                continue
            (db, block) = self.actions[action]
            action_obj = RobotMoveDBToTag(self.colors[db], block)
            self.action_pub.publish(action_obj)
            # Sleep to let action process
            rospy.sleep(C.Q_SLEEP_DURATION)
            # Sleep further for reward if needed
            while not self.reward:
                rospy.sleep(C.Q_SLEEP_DURATION)
            reward = self.reward.reward
            self.reward = None

            if self.update_q(curr_state, action, next_state, reward, alpha, gamma):
                last_update_iter = curr_iter
                self.publish_qmat()
            curr_state = next_state
            curr_iter += 1


            
    def reward_received(self, data):
        """
        Callback for reward events. Just saves the reward in class
        """
        # For unknown reason, multiple rewards are received sometimes for one action
        # if that action places the last DB in front of a block. 
        # The if check prevents this as only the first one is relevant to the action
        if not self.reward:
            self.reward = data

    
    def print_qmat(self):
        """
        Method for testing that just prints the rows of self.qmat
        """
        for row in self.qmat:
            print(row)


if __name__ == "__main__":
    learner = QLearning()
    learner.run()