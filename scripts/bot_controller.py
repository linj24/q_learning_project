#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# Robot states
# Initializing: Setting up everything 
# Learning: Phantom bot movement, q matrix
# Idle: Waiting for an action
# Scouting: Checking where each block/dumbbell is
# Finding_Dumbbell: Moving to dumbbell
# Grab Dumbbell: Using the arm to pick up the dumbbell
# Finding_Block: Moving to block with dumbbell
# Releasing Dumbbell: Using the arm to put down the dumbbell
STATE_INIT = 0
STATE_LEARN = 1
STATE_LOOP = 2


class Robot(object):
    
    def __init__(self):
        rospy.init_node('q_main')

    

    def update_state(self):
        pass
        
    def run(self):
        rospy.spin()


if __name__=="__main__":

    run()