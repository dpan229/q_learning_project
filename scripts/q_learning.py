#!/usr/bin/env python3

import rospy
import numpy as np
import os
import sys

from q_learning_project.msg import QMatrix, QMatrixRow, QLearningReward, RobotMoveObjectToTag

from copy import deepcopy
from random import choice

from time import sleep

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self, mode):
        # Initialize this node
        rospy.init_node("q_learning")

        self.mode = mode

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))


        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the pink, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        # initialize Q matrix to all zeros
        self.q_matrix = np.zeros((len(self.states), len(self.actions)))
        self.q_matrix_filename = "/home/ubuntu/catkin_ws/src/q_learning_project/q_matrix.csv"

        # set up publishers and subscribers
        self.matrix_publisher = rospy.Publisher('q_learning/q_matrix', QMatrix, queue_size=10)
        self.action_publisher = rospy.Publisher('q_learning/robot_action', RobotMoveObjectToTag, queue_size=10)
        reward_callback = self.receive_reward_train if mode == "train" else self.receive_reward_execute
        self.reward_listener = rospy.Subscriber('q_learning/reward', QLearningReward, reward_callback)

        # a matrix containing 1 row for each state and 1 column for each action.
        # state_map[s][a] tells you what state you will be in after executing action
        # `a` in state `s` (as indeces into self.states and self.actions). The value 
        # will be -1 if the action `a` is not possible in state `s`
        self.state_map = [[-1 for _ in self.actions] for _ in self.states]
        for state_id, row in enumerate(self.action_matrix):
            for state_2_id, action_id in enumerate(row):
                if action_id != -1:
                    self.state_map[state_id][int(action_id)] = state_2_id

        # index of the previous state and action
        self.previous_state_id = 0
        self.previous_action_id = 0

        # learning parameters
        self.alpha = 1.0
        self.gamma = 0.8

        # convergence variables and parameters
        self.old_q_matrix = deepcopy(self.q_matrix)
        self.check_iterations = 1000
        self.iteration_counter = 1

    def possible_actions(self, state_id):
        '''
        Returns a list of all possible actions from the given state
        (as indices into `self.actions`)
        '''
        return [i for i in range(len(self.actions)) if self.state_map[state_id][i] != -1]

    def send_action(self, action_id):
        '''
        Creates and publishes an object corresponding to the action
        at the given index into `self.actions`
        '''
        action = self.actions[action_id]
        print(f'Executing action: move {action["object"]} object to tag {action["tag"]}.')
        action_obj = RobotMoveObjectToTag(action["object"], action["tag"])
        self.action_publisher.publish(action_obj)

    def begin(self):
        '''
        Start training or executing the policy depending on mode
        '''
        if self.mode == "train":
            self.train()
        else:
            self.execute_policy()

    def train(self):
        print("Training")
        next_action_id = choice(self.possible_actions(self.previous_state_id))

        next_action = self.actions[next_action_id]
        next_action_obj = RobotMoveObjectToTag(next_action["object"], next_action["tag"])
        self.action_publisher.publish(next_action_obj)

    def receive_reward_train(self, data):
        #print(f"Recieved reward: {data.reward}")
        reward = data.reward
        new_state = self.state_map[self.previous_state_id][self.previous_action_id]

        # update the Q matrix given reward and previous state and action
        current_q_value = self.q_matrix[self.previous_state_id][self.previous_action_id]
        self.q_matrix[self.previous_state_id][self.previous_action_id] = current_q_value + self.alpha * (reward + self.gamma * np.max(self.q_matrix[new_state, :]) - current_q_value)
        #print(f"setting matrix[{self.previous_state_id}][{self.previous_action_id}] to {current_q_value + self.alpha * (reward + self.gamma * np.max(self.q_matrix[new_state, :]) - current_q_value)}")

        # check if the Q matrix has converged
        if self.iteration_counter % self.check_iterations == 0:
            # check if the old matrix is suffieciently close to the new matrix
            if np.sum(np.abs(self.q_matrix - self.old_q_matrix)) < 1.0:
                # matrix is converged, publish and save it and stop
                print("Matrix has converged")
                self.save_q_matrix()
                rospy.signal_shutdown('Training complete: Q matrix has converged')
                return
            else:
                self.old_q_matrix = deepcopy(self.q_matrix)
        self.iteration_counter += 1

        # send the next action if not converged and update previous state/action
        possible_actions = self.possible_actions(new_state)
        if possible_actions:
            next_action_id = choice(possible_actions)
            #print(f"state: {self.previous_state_id}, action: {self.previous_action_id}, new state: {new_state}")
            self.previous_state_id = new_state
            self.previous_action_id = next_action_id
        else:
            self.previous_state_id = 0
            next_action_id = choice(self.possible_actions(0))
            self.previous_action_id = next_action_id

        self.send_action(next_action_id)

    def save_q_matrix(self):
        '''
        Saves `self.q_matrix` to a file and publishes the matrix to the
        QMatrix topic
        '''
        np.savetxt(self.q_matrix_filename, self.q_matrix, delimiter=',')

        # convert q_matrix into a QMatrix object
        q_matrix_obj = QMatrix()
        q_matrix_obj.q_matrix = [QMatrixRow(list(row.astype(int))) for row in self.q_matrix]

        self.matrix_publisher.publish(q_matrix_obj)

    def read_q_matrix(self):
        '''
        Reads `self.q_matrix` from a file
        '''
        self.q_matrix = np.loadtxt(self.q_matrix_filename, delimiter=',')

    def policy(self, state_id):
        '''
        Given a state `state_id`, returns the action that the current
        policy thinks is the best
        '''
        return np.argmax(self.q_matrix[state_id, :])

    def execute_policy(self):
        self.read_q_matrix()

        if not self.possible_actions(self.previous_state_id):
            # we are in a final state
            rospy.signal_shutdown('Execution complete: final state reached')
            return

        next_action = self.policy(self.previous_state_id)
        self.previous_action_id = next_action
        self.send_action(next_action)


    def receive_reward_execute(self, data):
        new_state = self.state_map[self.previous_state_id][self.previous_action_id]

        if not self.possible_actions(new_state):
            # we have reached a final state
            rospy.signal_shutdown('Execution complete: final state reached')
            return

        next_action = self.policy(new_state)
        self.previous_state_id = new_state
        self.previous_action_id = next_action
        self.send_action(next_action)


if __name__ == "__main__":
    mode = "train" if "--train" in sys.argv else "exec"
    node = QLearning(mode)
    sleep(2.0)
    node.begin()
    rospy.spin()