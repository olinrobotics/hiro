#!/usr/bin/env python

"""
The RL sequencer for a 3x3 one layer, machine learning "sorter" of the center of the
structure of interest

by Kevin Zhang

this script is meant to be imported as a package to assembly_instructor.py

how it goes:
1. Loads fully trained Q Learning Table and other variables to start an environment
and run through it, as well sorting variables to keep track of the sorted instructions
2. Given a center of a structure's layer, run through the Q Learning Engine and find the
sequence using machine learning. This will involve conversions bewteen the cubes
and the one_hot mechanism used before
3. Return the sorted center and wait for another layer to sort

the code here is very similar to the code in easy_RL directory, for information
look there
"""


import rospy
import numpy as np
from std_msgs.msg import String, Int16
import time
from irl.msg import Cube, Structure
from cube import Digital_Cube
from RL import RL_brain
import argparse
import itertools
import cPickle as pickle
import rospkg



class Smart_Sequencer(object):
    """
    The RL sequencer that can sort a 3x3 layer in a structure
    """

    def __init__(self):
        # gets the Q Learning Table, fully trained
        print "LOADING MEMORY"
        rospack = rospkg.RosPack()
        self.PACKAGE_PATH = rospack.get_path("irl")
        with open(self.PACKAGE_PATH+"/projects/Planning/Assembler/final_memory.txt", 'rb') as f:
            q_table = pickle.load(f)
        print "DONE LOADING"

        #initializes RL_brain with trained Q Learning Table
        self.RL = RL_brain(q_table=q_table)

        # environment run through variables
        self.observation = None
        self.action = None
        self.reward = None
        self.env = None
        self.num_actions = 9
        self.agent_state = [0]*(self.num_actions*2)

        # sequencing variables
        self.sequence = []
        self.one_hot_mapping = {"[1, 1]":0,  "[2, 1]":1,  "[3, 1]":2, \
                                "[1, 2]":3,  "[2, 2]":4,  "[3, 2]":5, \
                                "[1, 3]":6,  "[2, 3]":7,  "[3, 3]":8}
        self.reverse_mapping = {0:[1, 1],  1:[2, 1],  2:[3, 1], \
                                3:[1, 2],  4:[2, 2],  5:[3, 2], \
                                6:[1, 3],  7:[2, 3],  8:[3, 3]}


    def convert_to_onehot(self, structure):
        """
        the RL engine needs the cubes to be in one_hot format, so this converts
        the cubes to a format the engine can recognize
        """

        self.agent_state = [0]*(self.num_actions*2)
        self.sequence = []
        for cube in structure:
            one_hot = self.one_hot_mapping[str([cube.x, cube.y])]

            # recreates the state used in training, which is how the RL engine sees the cubes
            self.agent_state[one_hot] = 1
            self.agent_state[one_hot+self.num_actions] = 1
        return self.agent_state[:]


    def find_cube(self, action, cubes):
        """
        converts back from one_hot to a real cube, so we can properly find and
        sort the real cubes, not just the one_hot format
        """

        x, y = self.reverse_mapping[action]
        for cube in cubes:
            if cube.x == x and cube.y == y:
                return cube

        return None


    def step(self, action):
        """
        step through the Q Learning Engine to figure out the correct sequence as
        the RL engine knows it, basically creates a pseudo environment and runs through it
        """

        # step once
        self.agent_state[action.astype(int)] = 0
        s_ = self.agent_state[:]

        # determine if done
        done = True if np.sum(self.agent_state[:9]) == 0 else False

        return s_, done


    def smart_sequence(self, environment):
        """
        the main sequencing method, basically runs through a mock trial with the RL engine
        and then finds the correct sequence
        """

        if environment == []: # edge case check
            return []
        cubes = environment[:]

        # convert to one_hot for RL run through
        self.observation = self.convert_to_onehot(environment)
        done_sequencing = False
        while not done_sequencing:

            # goes through choosing actions, storing the sequence, and then stepping, no learning
            self.action = self.RL.choose_action(str(self.observation))
            self.sequence.append(self.find_cube(self.action, cubes))
            self.observation, done_sequencing = self.step(self.action)

        return self.sequence



if __name__ == "__main__":
    main = Smart_Sequencer()
    main.run()
