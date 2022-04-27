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
