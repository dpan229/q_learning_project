#!/usr/bin/env python3

import rospy
import numpy as np
import os

from q_learning_project.msg import QMatrix, QLearningReward, RobotMoveObjectToTag

from copy import deepcopy
from random import choice

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

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

        # set up publishers and subscribers
        self.matrix_publisher = rospy.Publisher('q_learning/q_matrix', QMatrix, queue_size=10)
        self.action_publisher = rospy.Publisher('q_learning/robot_action', RobotMoveObjectToTag, queue_size=10)
        self.reward_listener = rospy.Subscriber('q_learning/reward', QLearningReward, self.receive_reward)

        self.state_map = [[-1 for _ in self.actions] for _ in self.states]
        for state_id, row in enumerate(self.action_matrix):
            for state_2_id, action_id in enumerate(row):
                self.state_map[state_id][int(action_id)] = state_2_id

        # index of the previous state and action
        self.previous_state_id = 0
        self.previous_action_id = 0

        # learning parameters
        self.alpha = 1.0
        self.gamma = 0.8

        self.old_q_matrix = deepcopy(self.q_matrix)
        self.check_iterations = 10
        self.iteration_counter = 1

    def train(self):
        next_action_id = choice(range(len(self.actions)))

        next_action = self.actions[next_action_id]
        next_action_obj = RobotMoveObjectToTag(next_action["object"], next_action["tag"])
        self.action_publisher.publish(next_action_obj)

    def receive_reward(self, data):
        print("Recieved reward")
        reward = data.reward
        new_state = self.state_map[self.previous_state_id][self.previous_action_id]

        # update the Q matrix given reward and previous state and action
        current_q_value = self.q_matrix[previous_state_id][previous_action_id]
        self.q_matrix[previous_state_id][previous_action_id] = current_q_value + self.alpha * (reward + self.gamma * max(self.q_matrix[new_state]) - current_q_value)

        # check if the Q matrix is converged
        if self.iteration_counter % self.check_iterations == 0:
            # check if the old matrix is suffieciently close to the new matrix
            if np.sum(self.q_matrix - self.old_q_matrix) < 1.0:
                # matrix is converged, publish and save it and stop
                print("Matrix has converged")
                self.save_q_matrix()
                return
            else:
                self.old_q_matrix = deepcopy(self.q_matrix)

        # send the next action if not converged and update previous state/action
        next_action_id = choice(range(len(self.actions)))
        self.previous_state_id = new_state
        self.previous_action_id = next_action_id

        next_action = self.actions[next_action_id]
        next_action_obj = RobotMoveObjectToTag(next_action["object"], next_action["tag"])
        self.action_publisher.publish(next_action_obj)

    def save_q_matrix(self):
        # save q_matrix to a file
        np.save_txt("q_matrix.csv", self.q_matrix, delimeter=',')

        # convert q_matrix into a QMatrix object
        q_matrix_obj = QMatrix()
        q_matrix_obj.q_matrix = [QMatrixRow(row) for row in self.q_matrix]

        self.matrix_publisher.publish(q_matrix_obj)
        return

if __name__ == "__main__":
    node = QLearning()
    node.train()