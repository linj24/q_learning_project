# q_learning_project

Enrique Collin and Jason Lin

## Q-learning algorithm
### Executing the Q-learning algorithm
We will encode the set of possible actions and states listed on the website and then follow the algorithm using the phantom robot node: until the Q-matrix converges, we will randomly select and perform an action and then update the Q-matrix using the reward published. 
We can test this by observing (1) how long it takes for the Q matrix to converge (2) whether the robot's behavior is as desired after the Q matrix converges (3) manually inspecting the Q matrix.
### Determining when the Q-matrix has converged
Determining when the Q-matrix has converged
We will determine that the Q-matrix has converged by keeping track of the iteration when it last updated. If it has not updated for some number of iterations, perhaps 50, we will decide that it has converged. 
We will test whether we chose too high/low a number of iterations by testing out different numbers and seeing if we need fewer/more iterations for the robot to achieve its goal using the ending Q matrix.
### Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward
Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward
We will continuously (1) determine the current state (2) determine the desired next state to maximize expected reward (3) use the action matrix entry to find the action that moves between these states. 
We will test this largely by whether the robot is able to achieve the goal of putting each dumbbell at the desired square.
## Robot perception
### Determining the identities and locations of the three colored dumbbells
To determine the identity we can use the /scan topic and /camera/rgb/image_raw to detect the direction of the dumbbells and the color of them. When the robot is initialized, we will record its odometry as the starting position. When the robot needs to search for a dumbbell, it will return t o this starting position and rotate until it detects the desired color.
We can test by observing the robot’s behavior in RViz and perhaps looking at the actual identities and positions of the dumbells sent by the ModelStates publisher. 
Determining the identities and locations of the three numbered blocks
### Determining the identities and locations of the three numbered blocks
We will procure a neural network designed to detect arabic numerals in images. When the robot is initialized, we will have it use the /scan topic to locate the positions of the three blocks and store them in memory. In front of each of the blocks, the robot will use the neural network to process the camera feed from /camera/rgb/image_raw and calculate the probability of that block being 1, 2, or 3.
We can test by observing the robot’s behavior in RViz and perhaps looking at the actual identities and positions of the blocks sent by the ModelStates publisher. 
## Robot manipulation & movement
### Picking up and putting down the dumbbells with the OpenMANIPULATOR arm
Picking up and putting down the dumbbells with the OpenMANIPULATOR arm
Once navigating to the location of the dumbbell, we will make sure we are directly facing it at a set distance (this standardizes the needed grab command). Then, we will use the MoveIt package (which can detect collisions) to grab the dumbbell using a pre-determined through trial/error grab setting. 
We can test by observing the robot’s behavior in RViz and editing behavior based on whether it is grabbing successfully or not. 
### Navigating to the appropriate locations to pick up and put down the dumbbells
We can navigate to the right location using the initial position of the robot and the /scan topic; we will use proportional control. If needed, we will use the fact that dumbbells have color to tell which direction has the dumbbells and which has the blocks.
We can test by observing the robot’s behavior in RViz and perhaps looking at the actual identities and positions of the blocks sent by the ModelStates publisher. 
## A brief timeline sketching out when you would like to have accomplished each of the components listed above.
Enrique-Qlearn 
Jason-Robot perception, arm manipulation
We will work independently at our own paces on the portions of the project mentioned above until next Wednesday’s in-class studio time. At that point we will ideally be nearly finished with our sections and will pair program/redistributed work on the remaining parts of the project. 
