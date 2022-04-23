#!/usr/bin/env python3

import rospy

from std_msgs.msg import Header
from q_learning_project.msg import ManipulatorAction, QLearningReward, QMatrix, QMatrixRow, RobotMoveDBToTag
import constants as C

import numpy as np
import os


# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class ManipulationPublisher(object):

    def __init__(self):
        # Initialize this node
        rospy.init_node('q_manipulation_publisher')

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

        # Values representing states
        self.ORIGIN = 0

        self.manipulator_action_pub = rospy.Publisher(C.MANIPULATOR_ACTION_TOPIC, ManipulatorAction, queue_size=10)
        rospy.Subscriber(C.MANIPULATOR_ACTION_TOPIC, ManipulatorAction, self.manipulator_action_received)
        # Variable used to track of last publisher ManipulatorAction has been received
        self.action_confirmation_received = False
        # Sleep for one second to setup subscriber and publishers
        rospy.sleep(1.0)

        self.qmat = np.loadtxt(os.path.dirname(__file__) + C.Q_MATRIX_FILE_PATH)
    
    
    def run(self):
        # Execute the path for maximum reward
        self.execute_path_for_reward()
        rospy.loginfo("DONE")


    def in_final_state(self, state):
        """
        Returns True if the state given has all 3 dumbbells at a block,
        False otherwise.
        """
        state_ls = self.states[state]
        return self.ORIGIN not in state_ls

    def is_valid_action(self, state, action):
        """
        Given a state and action, returns True
        if the action is valid for the current state,
        False otherwise.
        """
        (db, block) = self.actions[action]
        state_ls = self.states[state]
        return (block not in state_ls and state_ls[db] == self.ORIGIN)

    def get_action_end_state(self, state, action):
        """
        Given a state and action, returns the final state
        that will result if you perform the action.
        Raises RuntimeError if self.is_valid returns False
        """
        if not self.is_valid_action(state, action):
            raise RuntimeError("Given invalid action for state in get_action_end_state!")
        state_ls = self.states[state]
        (db, block) = self.actions[action]
        state_ls[db] = block
        return state_ls[0] + 4 * state_ls[1] + 16 * state_ls[2]


    def execute_path_for_reward(self):
        """
        After the QMatrix has converged, this function will send out 
        actions one at a time on the ManipulatorAction topic, waiting for confirmation
        that the action has been performed each time before sending the next. 
        The actions are chosen to maximize the likelihood of a reward based on QMatrix.
        """
        curr_state = 0
        while not self.in_final_state(curr_state):
            # We have not yet received confirmation for currenet action
            self.action_confirmation_received = False
            qvals = self.qmat[curr_state]
            # Do the work below in case of strange possible QMatrices
            # That won't actually show up in our problem
            best_actions = np.argsort(qvals)
            i = len(best_actions) - 1
            best_action = best_actions[i]
            while not self.is_valid_action(curr_state, best_action):
                if i == 0:
                    # There are no valid actions
                    return 
                else:
                    i -= 1
                    best_action = best_actions[i]

            (db, block) = self.actions[best_action]
            action_obj = ManipulatorAction()
            action_obj.is_confirmation = False
            action_obj.tag_id = block
            action_obj.robot_db = self.colors[db]
            self.manipulator_action_pub.publish(action_obj)
            curr_state = self.get_action_end_state(curr_state, best_action)
            # Now wait for response
            while not self.action_confirmation_received:
                rospy.sleep(2.0)


    def manipulator_action_received(self, data):
        """
        Callback for manipulator_action events. 
        These should only be received when the manipulator/movement code has 
        completed the last action WE published on this topic.
        """
        if data.is_confirmation:
            self.action_confirmation_received = True

    
if __name__ == "__main__":
    publisher = ManipulationPublisher()
    publisher.run()