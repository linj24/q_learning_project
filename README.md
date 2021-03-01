# q_learning_project 

Enrique Collin and Jason Lin
# Implementation Plan
## Q-learning algorithm
### Executing the Q-learning algorithm
- We will encode the set of possible actions and states listed on the website and then follow the algorithm using the phantom robot node: until the Q-matrix converges, we will randomly select and perform an action and then update the Q-matrix using the reward published. 
- We can test this by observing (1) how long it takes for the Q matrix to converge (2) whether the robot's behavior is as desired after the Q matrix converges (3) manually inspecting the Q matrix.
### Determining when the Q-matrix has converged
- We will determine that the Q-matrix has converged by keeping track of the iteration when it last updated. If it has not updated for some number of iterations, perhaps 50, we will decide that it has converged. 
- We will test whether we chose too high/low a number of iterations by testing out different numbers and seeing if we need fewer/more iterations for the robot to achieve its goal using the ending Q matrix.
### Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward
- We will continuously (1) determine the current state (2) determine the desired next state to maximize expected reward (3) use the action matrix entry to find the action that moves between these states. 
- We will test this largely by whether the robot is able to achieve the goal of putting each dumbbell at the desired square.
## Robot perception
### Determining the identities and locations of the three colored dumbbells
- To determine the identity we can use the `/scan` topic and `/camera/rgb/image_raw` to detect the direction of the dumbbells and the color of them. When the robot is initialized, we will record its odometry as the starting position. When the robot needs to search for a dumbbell, it will return to this starting position and rotate until it detects the desired color.
- We can test by observing the robot’s behavior in RViz and perhaps looking at the actual identities and positions of the dumbells sent by the `ModelStates` publisher. 
### Determining the identities and locations of the three numbered blocks
- We will procure a neural network designed to detect arabic numerals in images. When the robot is initialized, we will have it use the `/scan` topic to locate the positions of the three blocks and store them in memory. In front of each of the blocks, the robot will use the neural network to process the camera feed from `/camera/rgb/image_raw` and calculate the probability of that block being 1, 2, or 3.
- We can test by observing the robot’s behavior in RViz and perhaps looking at the actual identities and positions of the blocks sent by the `ModelStates` publisher. 
## Robot manipulation & movement
### Picking up and putting down the dumbbells with the OpenMANIPULATOR arm
- Once navigating to the location of the dumbbell, we will make sure we are directly facing it at a set distance (this standardizes the needed grab command). Then, we will use the MoveIt package (which can detect collisions) to grab the dumbbell using a pre-determined through trial/error grab setting. 
- We can test by observing the robot’s behavior in RViz and editing behavior based on whether it is grabbing successfully or not. 
### Navigating to the appropriate locations to pick up and put down the dumbbells
- We can navigate to the right location using the initial position of the robot and the `/scan` topic; we will use proportional control. If needed, we will use the fact that dumbbells have color to tell which direction has the dumbbells and which has the blocks.
- We can test by observing the robot’s behavior in RViz and perhaps looking at the actual identities and positions of the blocks sent by the `ModelStates` publisher. 
## Time
- Enrique: Qlearn 
- Jason: Robot perception, arm manipulation
- We will work independently at our own paces on the portions of the project mentioned above until next Wednesday’s in-class studio time. At that point we will ideally be nearly finished with our sections and will pair program/redistributed work on the remaining parts of the project. 

# Writeup
## Objectives description (2-3 sentences): Describe the goal of this project.
## High-level description (1 paragraph): At a high-level, describe how you used reinforcement learning to solve the task of determining which dumbbells belong in front of each numbered block.
## Q-learning algorithm description: Describe how you accomplished each of the following components of the Q-learning algorithm in 1-3 sentences, and also describe what functions / sections of the code executed each of these components(1-3 sentences per function / portion of code):
### Selecting and executing actions for the robot (or phantom robot) to take
### Updating the Q-matrix
### Determining when to stop iterating through the Q-learning algorithm
### Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot
## Robot perception description: Describe how you accomplished each of the following components of the perception elements of this project in 1-3 sentences, any online sources of information/code that helped you to recognize the objects, and also describe what functions / sections of the code executed each of these components (1-3 sentences per function / portion of code):
### Identifying the locations and identities of each of the colored dumbbells
### Identifying the locations and identities of each of the numbered blocks
## Robot manipulation and movement: Describe how you accomplished each of the following components of the robot manipulation and movement elements of this project in 1-3 sentences, and also describe what functions / sections of the code executed each of these components (1-3 sentences per function / portion of code):
### Moving to the right spot in order to pick up a dumbbell
### Picking up the dumbbell
### Moving to the desired destination (numbered block) with the dumbbell
### Putting the dumbbell back down at the desired destination
## Challenges (1 paragraph): Describe the challenges you faced and how you overcame them.
## Future work (1 paragraph): If you had more time, how would you improve your implementation?
## Takeaways (at least 2 bullet points with 2-3 sentences per bullet point): What are your key takeaways from this project that would help you/others in future robot programming assignments working in pairs? For each takeaway, provide a few sentences of elaboration.
