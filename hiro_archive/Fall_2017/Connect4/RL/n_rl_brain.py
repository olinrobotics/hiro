#!/usr/bin/env python

import numpy as np
import random
import time
from collections import defaultdict

class QLearningTable:
    def __init__(self, actions, learning_rate=0.004, reward_decay=0.9, e_greedy=0.95, q_table=None):
        self.actions = actions  # a list
        self.lr = learning_rate
        self.gamma = reward_decay
        self.epsilon = e_greedy
        self.q_table = {} if q_table is None else q_table

    def choose_action(self, observation):
        # self.check_state_exist(observation)
        # action selection
        if np.random.uniform() > self.epsilon:
            # choose best action
            try:
                state_action = self.q_table[observation]
            except KeyError:
                self.q_table[observation] = np.zeros(len(self.actions))
                state_action = self.q_table[observation]

            state_action = np.argwhere(state_action==np.amax(state_action)).flatten().tolist()
            action = np.random.choice(state_action)
        else:
            # choose random action
            action = np.random.choice(self.actions)
        return action

    def learn(self, s, a, r, s_):
        # self.check_state_exist(s)
        try:
            check = self.q_table[s]
        except KeyError:
            self.q_table[s] = np.zeros(len(self.actions))
        try:
            check = self.q_table[s_]
        except KeyError:
            self.q_table[s_] = np.zeros(len(self.actions))
            
        # self.check_state_exist(s_)
        q_predict = self.q_table[s][a]
        q_target = r + self.gamma * np.amax(self.q_table[s_])
        self.q_table[s][a] += self.lr * (q_target - q_predict)  # update

    def update_params(self):
        self.lr *= (1-self.lr *.000025)
        self.epsilon *= (1-self.epsilon*.0000009)

    # def check_state_exist(self, state):
    #     if state not in self.q_table:
    #         # append new state to q table
    #         self.q_table[state] = np.zeros(len(self.actions))
