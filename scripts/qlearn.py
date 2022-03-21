#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Header
from q_learning_project.msg import RobotMoveDBToTag
from q_learning_project.msg import QLearningReward
from q_learning_project.msg import QMatrixRow
from q_learning_project.msg import QMatrix
from q_learning_project.msg import ManipulatorAction
from constants import MANIPULATOR_ACTION_TOPIC

import numpy as np
import random

class QLearn():
    # Action is of the form (dumbell to move, block to move to)
    # There are 3 dumbells and 3 blocks so 9 actons
    NUM_ACTIONS = 9 
    # States are of the form (red DB location, green DB location, blue DB location)
    # There are 4 locations (origin, block1, block2, block3) 
    # So, 4^3 = 64 states
    NUM_STATES = 64 

    # Values representing states
    ORIGIN = 0
    BLOCK1 = 1
    BLOCK2 = 2
    BLOCK3 = 3

    # Values representing colors
    RED = 0
    GREEN = 1
    BLUE = 2

    def __init__(self):
        # initialize this node
        rospy.init_node('qlearn')

        # Setup publisher for changes to QMatrix and when we want to send an action
        self.qmat_pub = rospy.Publisher("q_learning/q_matrix", QMatrix, queue_size=10)
        self.action_pub = rospy.Publisher("q_learning/robot_action", RobotMoveDBToTag, queue_size=10)
        # Setup subcriber for rewards
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.reward_received)

        self.manipulator_action_pub = rospy.Publisher(MANIPULATOR_ACTION_TOPIC, ManipulatorAction, queue_size=10)
        rospy.Subscriber(MANIPULATOR_ACTION_TOPIC, ManipulatorAction, self.manipulator_action_received)
        # Variable used to track of last publisher ManipulatorAction has been received
        self.action_confirmation_received = False
        # Sleep for one second to setup subscriber and publishers
        rospy.sleep(1.0)
        self.qmat = np.zeros((self.NUM_STATES, self.NUM_ACTIONS), dtype=int)
        self.init_action_mat()
        self.reward = None
    
    
    def run(self):
        # Converge the matrix
        self.do_qlearn()
        # Execute the path for maximum reward
        self.execute_path_for_reward()

    def get_action_number(self, DB_color, block_number):
        """
        Gives the number representing the action:
        move DB with DB_color to block with block_number
        """
        if block_number not in [self.BLOCK1, self.BLOCK2, self.BLOCK3]:
            raise RuntimeError("get_action_number given invalid block number!")
        if DB_color not in [self.RED, self.GREEN, self.BLUE]:
            raise RuntimeError("get_action_number given invalid DB color value!")

        action = DB_color * 3
        action += block_number - 1
        return action
    

    def action_to_desc(self, action):
        """
        Takes a number representing an action and returns a tuple 
        of form (DB_color, block_number) representing it.
        If DB_color not in [self.BLUE, self.GREEN, self.RED] raises a 
        RuntimeError. 
        """
        block_num = action % 3
        DB_color = (action - block_num) // 3
        block_num += 1
        if DB_color == self.BLUE:
            color = "blue"
        elif DB_color == self.GREEN:
            color = "green"
        elif DB_color == self.RED:
            color = "red"
        else:
            raise RuntimeError("action_to_desc found invalid DB color!")
        return (color, block_num)
        

    def state_num_to_ls(self, state_num):
        """
        Maps a state_num in [0, 63] to a list of form
        [red_DB_state, green_DB_state, blue_DB_state]
        where those values are in 
        [self.ORIGIN, self.BLOCK1, self.BLOCK2, self.BLOCK3]
        """
        red = state_num % 4
        green = (state_num // 4) % 4
        blue = (state_num // 16) % 4
        return [red, green, blue]
    

    def state_ls_to_num(self, state_ls):
        """
        Given a state_ls of form returned by state_num_to_ls,
        returns the equivalent state_num.
        """
        state = 0
        state += state_ls[0]
        state += 4 * state_ls[1]
        state += 4 * 4 * state_ls[2]
        return state
        

    def get_action_for_state_change(self, state1_num, state2_num):
        """
        Returns -1 if there is no action that moves from state1_num to state2_num
        or the number of the action that makes that move otherwise.
        More specifically, -1 will be returned if:
        -Changing states requires moving > 1 DB
        -Changing states requires moving a DB that is not at origin
        -The states are the same
        """
        state1_ls = self.state_num_to_ls(state1_num)
        state2_ls = self.state_num_to_ls(state2_num)
        # If the states are the same return -1; no action needed
        if state1_ls == state2_ls:
            return -1
        diff = [state2_ls[i] - state1_ls[i] for i in range(3)]
        changed_DB = -1
        for (i, delta) in enumerate(diff):
            if delta != 0:
                # There is no action to change state of multiple dumbells  
                # So  if two states differ in more than one index return -1 for invalid
                if changed_DB != -1:
                    return -1
                changed_DB = i

        # Initial DB location and final DB location for the DB that moved
        DB_i, DB_f = state1_ls[changed_DB], state2_ls[changed_DB]
        # We consider the following moves invalid:
        # Moving a DB from anywhere but origin 
        # (note that since DB_f != DB_f the above ensures we move to a block)
        # Moving a DB to a block that already has one DB
        if DB_i != self.ORIGIN or state2_ls.count(DB_f) > 1:
            return -1
        return self.get_action_number(changed_DB, DB_f)


    def init_action_mat(self):
        """
        Initializes self.action_mat by calling get_action_for_state_change
        on every possible two states. 
        """
        NUM_STATES = self.NUM_STATES
        self.action_mat = np.zeros((NUM_STATES, NUM_STATES), dtype=int)
        for s1 in range(NUM_STATES):
            for s2 in range(NUM_STATES):
                self.action_mat[s1][s2] = self.get_action_for_state_change(s1, s2)


    def get_rand_action(self, start_state):
        """
        Returns a (rand_action, final_state) uniformly sampled from the valid actions
        starting at start_state
        If there are no valid actions, return -1
        """
        # actions stores a list of actions indexed by end state
        actions = self.action_mat[start_state]
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
                rospy.sleep(1)
                curr_state = 0
                continue
            (db, block) = self.action_to_desc(action)
            action_obj = RobotMoveDBToTag(db, block)
            self.action_pub.publish(action_obj)
            # Sleep to let action process
            rospy.sleep(1.0)
            # Sleep further for reward if needed
            while not self.reward:
                rospy.sleep(1)
            reward = self.reward.reward
            self.reward = None

            if self.update_q(curr_state, action, next_state, reward, alpha, gamma):
                last_update_iter = curr_iter
                self.publish_qmat()
            curr_state = next_state
            curr_iter += 1

    def in_final_state(self, state):
        """
        Returns True if the state given has all 3 dumbbells at a block,
        False otherwise.
        """
        state_ls = self.state_num_to_ls(state)
        return self.ORIGIN not in state_ls

    def is_valid_action(self, state, action):
        """
        Given a state and action, returns True
        if the action is valid for the current state,
        False otherwise.
        """
        (db, block) = self.action_to_desc(action)
        state_ls = self.state_num_to_ls(state)
        mapping = {"red": self.RED, "green": self.GREEN, "blue": self.BLUE}
        return (block not in state_ls and state_ls[mapping[db]] == self.ORIGIN)

    def get_action_end_state(self, state, action):
        """
        Given a state and action, returns the final state
        that will result if you perform the action.
        Raises RuntimeError if self.is_valid returns False
        """
        if not self.is_valid_action(state, action):
            raise RuntimeError("Given invalid action for state in get_action_end_state!")
        state_ls = self.state_num_to_ls(state)
        (db, block) = self.action_to_desc(action)
        mapping = {"red": self.RED, "green": self.GREEN, "blue": self.BLUE}
        state_ls[mapping[db]] = block
        return self.state_ls_to_num(state_ls)


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

            (db, block) = self.action_to_desc(best_action)
            action_obj = ManipulatorAction()
            action_obj.is_confirmation = False
            action_obj.block_id = block
            action_obj.robot_db = db
            self.manipulator_action_pub.publish(action_obj)
            curr_state = self.get_action_end_state(curr_state, best_action)
            # Now wait for response
            while not self.action_confirmation_received:
                rospy.sleep(2.0)


            
    def reward_received(self, data):
        """
        Callback for reward events. Just saves the reward in class
        """
        # For unknown reason, multiple rewards are received sometimes for one action
        # if that action places the last DB in front of a block. 
        # The if check prevents this as only the first one is relevant to the action
        if not self.reward:
            self.reward = data

    def manipulator_action_received(self, data):
        """
        Callback for manipulator_action events. 
        These should only be received when the manipulator/movement code has 
        completed the last action WE published on this topic.
        """
        if data.is_confirmation:
            self.action_confirmation_received = True

    
    def print_qmat(self):
        """
        Method for testing that just prints the rows of self.qmat
        """
        for row in self.qmat:
            print(row)

    def state_to_desc(self, state):
        """
        Method a for testing that converts a state number to a description of each
        DB's location and returns that as a string. 
        """
        ls = self.state_num_to_ls(state)
        mapping = {self.ORIGIN:"ORIGIN", self.BLOCK1: "BLOCK1", self.BLOCK2: "BLOCK2", self.BLOCK3: "BLOCK3"}
        return f"red: {mapping[ls[0]]}, green: {mapping[ls[1]]}, blue: {mapping[ls[2]]}"

if __name__ == "__main__":
    learner = QLearn()
    learner.run()