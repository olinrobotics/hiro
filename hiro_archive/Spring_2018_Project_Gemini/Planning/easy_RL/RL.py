#!/usr/bin/env python

"""
A Q Learning implementation engine that uses adaptive learning rate and sharp e_greedy

by Kevin Zhang

this script will be imported as a package to sequencer_RL

the basic run down is as follows:
1. initialize the q_table as empty (usually, unless otherwise given), and initialize
the other variables as well, including learning_rate, e_greedy, etc.
2. given a state, the Q Learning engine will choose an action, given its dictionary
state, with a probability of choosing a random action to explore more states
3. env_RL will return a reward based on the action chosen, and the Q Learning engine
will learn based on that reward, updating its values for the state/action pair
4. this continue until hopefully convergence is found
"""

import rospy
import numpy as np
import pandas as pd
import time


class RL_brain(object):
    """
    The RL_brain class, which holds the Q Learning Engine and the memory which is
    the Q Learning Table
    """

    def __init__(self, learning_rate=.02, reward_decay=0.9, e_greedy=1.0, q_table=None):
        self.encoded_action = None # the encoded action 0-num_actions based on the state values
        self.real_action = None # the real action based on the actual cube positions in the state
        self.lr = learning_rate # the learning rate
        self.gamma = reward_decay  # the "freshness" of memory
        self.epsilon = e_greedy # exploration vs. exploitation
        self.q_table = {} if q_table is None else q_table  # the Q Learning table
        self.q_table_counter = {} # the counter table used for adaptive learning rate

    def choose_action(self, observation):
        """
        Choose an action based on a state, according the table's Q Learning values
        there is also a probability it will choose a random action, in an effort
        to further explore the state space
        """

        # parse the obesrvation
        eval_func = eval("lambda: " + observation)
        state = eval_func()
        num_actions = np.count_nonzero(state[:9])

        # check if state exists, else make a new one
        try:
            state_action = self.q_table[observation]
        except KeyError:
            self.q_table[observation] = np.zeros(num_actions)
            state_action = self.q_table[observation]

        # check if state exists for counters, else make a new one
        try:
            counters = self.q_table_counter[observation]
        except KeyError:
            self.q_table_counter[observation] = np.zeros(num_actions)

        # choose action
        if np.random.uniform() > self.epsilon:
            # choose best action
            state_action = np.argwhere(state_action==np.amax(state_action)).flatten().tolist()
            self.encoded_action = np.random.choice(state_action)
        else:
            # choose random action
            self.encoded_action = np.random.choice(np.arange(num_actions))

        # find the real action for the state space according to the state
        self.real_action = np.nonzero(state)[0][self.encoded_action]

        # increment for adaptive learning rate
        self.q_table_counter[observation][self.encoded_action] += 1

        return self.real_action


    def learn(self, s, r, s_):
        """
        update Q Learning table based on reward
        """

        # check if state exists, else make it
        try:
            check = self.q_table[s]
        except KeyError:
            eval_func = eval("lambda: " + s)
            num_actions = np.count_nonzero(eval_func()[:9])
            self.q_table[s] = np.zeros(num_actions)

        # check if counter exists, else make it
        try:
            counters = self.q_table_counter[s]
        except KeyError:
            eval_func = eval("lambda: " + s)
            num_actions = np.count_nonzero(eval_func()[:9])
            self.q_table_counter[s] = np.zeros(num_actions)

        # check if the next state from the action exists, else make it
        try:
            check = self.q_table[s_]
        except KeyError:
            eval_func = eval("lambda: " + s_)
            num_actions = np.count_nonzero(eval_func()[:9])
            num_actions = num_actions if num_actions > 0 else 1 # if we reach the final state
            self.q_table[s_] = np.zeros(num_actions)

        # update the Q Learning Value according to the reward
        q_predict = self.q_table[s][self.encoded_action]
        q_target = r + self.gamma * np.amax(self.q_table[s_])
        learning_rate = self.q_table_counter[s][self.encoded_action] # adaptive learning rate per state/action pair
        self.q_table[s][self.encoded_action] += (1.0/learning_rate) * (q_target - q_predict)  # update



    def update_params(self):
        """
        updates the exploration vs exploitation rate over time
        """

        self.epsilon *= (1-self.epsilon*.000003)
