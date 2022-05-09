# Q-Learning Project

## Group Members

David Pan, William Zeng

## Implementation Plan

### Executing the Q-learning algorithm

The Q-learning algorithm will be done using 64 different states corresponding
to the possible sets locations of the three colored objects and 9 different 
actions in each state corresponding to placing one of the objects in front of
one of the tags. The training phase of the algorithm will be using
`virtual_reset_world` to get the rewards quickly and run through many
iterations until the Q-matrix is converged.

### Determining when the Q-matrix has converged

We will consider the Q-matrix to have converged when it has not changed
substantially for some large number of iterations. We will test our
implementation by running the training phase many times in the same
conditions and seeing if the converged matrix is the same every time.

### Once the Q-matrix has converged, determining which actions the robot should take to maximize expected reward

Once the Q-matrix has converged, the action that the robot should take to
maximize its expected reward is the action with the greatest Q-value in the
robot's current state.

### Determining the identities and locations of the three colored objects

We will determine the locations of the colored objects using both the robot's
LiDAR and RGB camera. The objects should be closer to the robot than the
surrounding regions in its field of view, and they will be detectable on the
camera feed as a large region of pixels all close to the corresponding color.
We will test our implementation by placing the objects in different positions
around the robot and making the robot rotate until it detects them.

### Determining the identities and locations of the three AR tags

We will determine the locations of the AR tags by feeding images from the
robot's camera into ArUco library's tag detection function. We will test this
in the same way as the above.

### Picking up and putting down the colored objects with the OpenMANiPULATOR arm

We will implement this using the GUI to find the appropriate joint and grapper
motions to pick up and put down the objects from a specific distance. We will
execute these motions using the MoveIt package.

### Navigating to the appropriate locations to pick up and put down the colored objects

When we want to the robot to move to a specific colored object or tag, we will 
determine which direction to move by making the robot rotate until it sees
the object and then move towards it until it gets close enough to it, which
we will detect using the LiDAR. 

### Timeline

Week 1: Implementing Q-learning

Week 2: Library usage, vision and robot movement 

Finish project end of week 2.

## Writeup

### Objective Description

The goal of this project is to implement a behavior that allows the robot to
figure out how to organize the objects in its environment using reinforcement
learning. This involves implementing the Q-learning algorithm, as well as 
elements of perception and control to allow the robot to recognize and
manipulate other objects.

### High-level Description

The robot learns which colored object should go in front of each AR tag via
the Q-learning algorithm. We first train the robot's behavior in a simulated
environment where a large reward is given if all three objects are in front
of the correct tags. Each possible action in every state has an associated 
value, and with each action that the robot takes, the corresponding value 
is updated based on the reward received, as well as the largest value among
the possible actions from the resulting state. The training phase consists
of the robot taking random actions until all of the values have converged. 
When the training is complete, the best course of action learned by the 
algorithm will involve placing the objects in front of the correct tags.

### Q-learning Algorithm Description

#### Selecting and execturing actions

During training, the robot chooses actions to take uniformly randomly among
all of the possible actions in its current state. If there are no possible
actions in the current state, the world is reset to the starting state and 
the robot picks a random action from this state in the same way. The chosen 
action is executed by sending a `RobotMoveObjectToTag` message to the 
`q_learning/robot_action` topic, which is received by the `virtual_reset_world` 
node. This component is implemented in the `QLearning.receive_reward_train` 
callback function in `q_learning.py`, and uses the `possible_actions` and 
`send_action` helper functions.

#### Updating the Q-matrix

When the robot receives the reward for its last action, the Q-matrix entry
corresponding to executing that action in the previous state is updated
based on the size of the reward and the largest Q-matrix entry in the row
corresponding to the new state. We keep track of the previous state and
action by storing indices into `self.states` and `self.actions` respectively.
This is also implemented in the `QLearning.receive_reward_train` callback
function.

#### Determining when to stop iterating through the Q-learning algorithm

To determine when the Q-matrix has converged, after every 1000 actions we 
compare the current Q-matrix to a copy of the old Q-matrix from 1000 actions
ago. If the sum of the absolute values of the differences between the
corresponding matrix elements is less than 1, then we consider the matrix 
to have converged and end the training. This is also implemented in the
`QLearning.receive_reward_train` callback function with some variables set up
in the `QLearning` initialization function.

### Executing the path most likely to lead to a reward after the Q-matrix has converged

After the training is complete, the optimal path is executed by always
picking the action that has the largest associated value from the current 
state in the Q-matrix. This is implemented in the `receive_reward_execute`
callback function in `q_learning.py`.

### Robot Perception Description

## Identifying the locations and identities of each of the colored objects:
* We identified a range of colors for pink, green, and blue first. This required encompassing a range where the robot can recognize when the object is in the shadow to when the object is in the light. After it identifies an object with this range, it finds the middle of all the pixels that contain this color and moves towards that pixel with the incorporation of sound

Identifying the locations and identities of each of the AR tags:
* To identify the AR tags, we used the provided library RobotMoveObjectToTag. Like the colored object, we identified the middle of the tag and moved towards it with the consideration of noise.

### Robot Manipulation and Movement
Robot manipulation and movement: Describe how you accomplished each of the following components of the robot manipulation and movement elements of this project in 1-3 sentences, and also describe what functions / sections of the code executed each of these components (1-3 sentences per function / portion of code):
## Moving to the right spot in order to pick up a colored object
* implemented in `move_object()` on line 146
* It first rotates masks the robot's camera so it only recognizes a range of colors in `image_callback` (line 116). After seeing the colors that fall in that range, it averages all the pixels with colors in that range and finds where the center of the color object is (line 146). Then it moves towards it until it reaches a certain distance (1.5m) to stop right in front of the object to pick it up.

## Picking up the colored object
* implemented in `claw_grab()` on line 205.
* We used moveit_commander in order to rotate the joints so that it ins in position to grab. It will first extend its arm (line 208). Once it reaches out far enough to cover the distance between the bot and the object, it grippens its claw (line line 217) and lifts it up out of camera view (line 224).

## Moving to the desired destination (AR tag) with the colored object
* implemented in `move_object()` on line 173.
* After grabbing the object, the robot recognizes the tag using RobotMoveToTag, identify the center of the tag (line 105), and uses Twist to move accordingly to which tag the q_learning tells the robot to drop the object at.

## Putting the colored object back down at the desired destination
* implemented in `claw_open()` on line 230.
* like `claw_grab()`, we used moveit_commander to rotate the joints and control claw. It first brings the arm down (line 232), opens the gripper to drop the object (line 240) and have the robot retreive it's arm so that it doesn't hit the object when it rotates for the next task.

#### Challenges
The challenges we faced during this project is organizing the joint angles such that it wouldn't get in the way of the camera, setting up the camera for the robot for debugging, and understanding the data structures needed for certain variables. For the joint angles, we tested both positive and negative angles to see how the robot arm would react until we know how the joints move. As for the camera, we used lab b to reflect what we needed to include in our code but had to do some adjustments based on the TA's response to use on slack as for what was going on and why our camera did not work. Lastly, for the data structures, we simply looked at error which referred us to the line where a data structure did not work. Then we looked through documentation and reasoning for what to change or assert our variables to be and change it accordingly.

#### Future Work
If I had more time, I would figure out a way to help the robots recognize the tags at sharper angles. As of now, the robot does not seem to be able to recognize the tags when it's at a close to 30 degree angle.

#### Takeaways
Takeaways (at least 2 bullet points with 2-3 sentences per bullet point): What are your key takeaways from this project that would help you/others in future robot programming assignments working in pairs? For each takeaway, provide a few sentences of elaboration.

#### GIF
![q_learning](mover.gif)
