#!/usr/bin/env python

"""
The env_RL program is meant to act as a digital environment for the RL agent to run
through and helps decide the reward based on the actions taken by the agent.

Kevin Zhang

This script is meant to be imported as a package into sequencer_RL.py

the basic functionality is as follows:

1. first initializes the environment, which consists of a one-hot mapping of the built
structure from digital_env.py, this can include resetting as well. also will
create a target which represents the sorted instructions as per the pattern sorter,
which is what we're trying to make the RL agent learn
2. after that, the sequencer_RL will call step on the environment given the action
the RL agent decided to take, which will then run the action through the environment
and return the results of running that action. this continues until the environment
decides that the run is over, either because the agent reached the end or it messed up
3. it will then wait for a prompt to reset and build a new environment, at which the
process starts over

"""


import rospy
import numpy as np
from std_msgs.msg import String, Int16
import time
from irl.msg import Grid_Cube, Grid_Structure
from cube import Digital_Cube
from digital_env import Environment
import itertools


class RL_environment(object):
    """
    the RL_environment class, which holds functionality to run the environment that
    the RL agent moves around in and determines rewards and the status of the current
    run based on how the agent moves
    """


    def __init__(self):
        self.env = Environment() # this calls digital_env
        self.action_space = range(9)
        self.num_actions = len(self.action_space)
        self.target = None  # this is the target that the RL agent wants to go to
        self.agent_state = [0]*(self.num_actions*2) # this is the state that the agent is currently in
        self.one_hot_mapping = {"[0, 0]":0,  "[1, 0]":1,  "[2, 0]":2, \
                                "[0, 1]":3,  "[1, 1]":4,  "[2, 1]":5, \
                                "[0, 2]":6,  "[1, 2]":7,  "[2, 2]":8}

                                # this is the one hot mapping for positions of cubes and their
                                # relation to the agent state

    def build_maze(self):
        """
        wrapper for reset
        """

        self.reset()


    def reset(self):
        """
        reset means to create a new digital env and set the target and the starting
        RL agent state, then return that to sequencer_RL
        """

        structure = self.env.create_a_struct() # this makes a new structure from digital_env
        self.target = []
        self.agent_state = [0]*(self.num_actions*2)
        for cube in structure:
            one_hot = self.one_hot_mapping[str([cube.x, cube.y])]

            # agent state holds the starting state twice, one that we run through, and the other
            # that acts as a identifier
            self.agent_state[one_hot] = 1
            self.agent_state[one_hot+self.num_actions] = 1
            self.target.append(one_hot)

        return self.agent_state[:]


    def step(self, action):
        """
        steps through the agent_state based on action taken by the RL agent
        then determines rewards and whether it's done based on a reward function
        """

        # step through with the action
        self.agent_state[action.astype(int)] = 0
        s_ = self.agent_state[:] # set the next state

        # the reward function
        if action != self.target[0]:
            reward = -1
            done = True
        elif action == self.target[0]:
            self.target = self.target[1:]
            reward = 1 if len(self.target) == 0 else 0
            done = True if len(self.target) == 0 else False

        return s_, reward, done


if __name__=="__main__":
    test = RL_environment()
    test.reset()
