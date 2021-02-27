#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Header
from q_learning_project.msg import RobotMoveDBToBlock
from q_learning_project.msg import QLearningReward
from q_learning_project.msg import QMatrixRow
from q_learning_project.msg import QMatrix

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
        self.qmat_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10)
        # Setup subcriber for rewards
        rospy.Subscriber("/q_learning/QLearningReward", QLearningReward, self.reward_received)
        self.qmat = np.zeros((NUM_STATES, NUM_ACTIONS))
        self.init_action_mat()
        self.reward = None
        self.do_qlearn()
    
    
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
    
    def action_num_to_obj(self, action):
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
            raise RuntimeError("action_num_to_obj found invalid DB color!")
        action_obj = RobotMoveDBToBlock(color, block_num)
        return action_obj
        


    def state_num_to_ls(self, state_num):
        """
        Maps a state_num in [0, 63] to a list of 
        [red_DB_state, green_DB_state, blue_DB_state]
        where those values are in [0, 4] to indicate origin or at one of the 3 blocks
        """
        red = state_num % 4
        green = (state_num // 4) % 4
        blue = (state_num // 16) % 4
        return [red, green, blue]
        

    def get_action_for_state_change(self, state1_num, state2_num):
        state1_ls = self.state_num_to_ls(state1_num)
        state2_ls = self.state_num_to_ls(state2_num)
        # If the states are the same return -1; no action needed
        if state1_ls == state2_ls:
            return -1
        diff = [state2_ls[i] - state1_ls[i] for i in range(3)]
        changed_DB = -1
        blocks_in_use = set()
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
        return self.get_action_number(DB_changed, DB_f)


    def init_action_mat(self):
        # action_mat[curr_state][desired_next_state] = action to get to desired state
        # -1 if transition is impossible
        NUM_STATES = self.NUM_STATES
        self.action_mat = np.zeros((NUM_STATES, NUM_STATES))
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
        final_state = random.randint(len(valid_actions))
        return (valid_actions[final_state], final_state)
        

    def do_qlearn(self):
        last_update_iter = curr_iter = 0
        # state 0 is everything at origin
        curr_state = 0
        converge_threshold = 50
        alpha = 1
        # Consider fine-tuning gamma choice
        gamma = 1
        # Converge after we have not updated qmat for converge_threshold iterations
        while curr_iter - last_update_iter < converge_threshold:
            (action, next_state) = self.get_rand_action(curr_state)
            if action == -1:
                # No valid actions left; all DBs at a block and this iteration is over
                print("No valid actions left. Sleeping for a second; world should reset")
                rospy.sleep(1)
                curr_state = 0
                continue

            action_obj = self.action_num_to_obj(action)
            self.action_pub.publish(action_obj)
            # Delay after sending robot action to phantom to give it time between actions
            # and wait for reward
            while not self.reward:
                rospy.sleep(1)
            reward = self.reward.reward
            self.reward = None
            curr_q = self.qmat[curr_state][action]
            next_max_Q = max(self.qmat[next_state])
            self.qmat[curr_state][action] = curr_q + alpha * (reward + gamma * next_max_Q - curr_q)
            
            tolerance = 0.01
            if abs(self.qmat[curr_state][action] - curr_q) > tolerance:
                # Check that an update occurred
                last_update_iter = curr_iter
            curr_state = next_state
            curr_iter += 1
            
            
    def reward_received(self, data):
        self.reward = data




        
