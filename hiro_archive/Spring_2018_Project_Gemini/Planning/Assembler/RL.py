#!/usr/bin/env python

"""
The Q Learning Engine, fully trained and integrated into the assembler

this script is meant to be imported as a package by sequence_RL.py

How it goes:
1. It will initialize everything in full "testing" mode, with a fully trained Q Learning Table
and full exploitation
2. Given a state, it will look in its Q Learning Table to find the best action and return it
3. Assuming that the Q Learning Table is fully trained, that's all this thing needs to do
it's mainly an encapsulation and wrapper to use a fully trained Q Learning Engine

"""

import rospy
import numpy as np
import pandas as pd
import time


class RL_brain(object):
    """
    Rl brain is a wrapper to use a fully trained Q Learning dictionary
    """

    def __init__(self, q_table=None):
        self.encoded_action = None   # the encoded action 0-num_actions based on the state values
        self.real_action = None    # the real action based on the actual cube positions in the state
        self.q_table = {} if q_table is None else q_table # the fully trained Q Learning Table

    def choose_action(self, observation):
        """
        chooses the best action for a given observation using the fully trained
        Q Learning dictionary
        """

        # parses observation
        eval_func = eval("lambda: " + observation)
        state = eval_func()
        num_actions = np.count_nonzero(state[:9])

        # checks if state exists (and it should for all things)
        try:
            state_action = self.q_table[observation]
        except KeyError:
            self.q_table[observation] = np.zeros(num_actions)
            state_action = self.q_table[observation]

        # choose best action
        state_action = np.argwhere(state_action==np.amax(state_action)).flatten().tolist()
        self.encoded_action = np.random.choice(state_action)

        self.real_action = np.nonzero(state)[0][self.encoded_action]

        return self.real_action
