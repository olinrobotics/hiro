#!/usr/bin/env python

"""
The RL Brain

by Kevin Zhang

This the main class that trains and validates the Q Learning Engine

Run this with

rosrun sequencer_RL.py -t

How it goes:
1. Initialize a RL environment and a RL Q Learning Engine
2. For a large number of trials (the training), reset the environment, choose an action,
step through the environment with that action, and learn with the RL Q Learning Engine based on the
reward
3. Once the run finishes (it succeeded or failed), start again and learning again
4. Once every X trials, run a test of 100 to see how accurate it's looking at that time

You can then validate the finished training set using test(), which performs the same
actions above and outputs the accuracy with an additional verbose output so you can
see what's going on, but without any learning involved.

"""

import rospy
import numpy as np
from std_msgs.msg import String, Int16
import time
from irl.msg import Cube, Structure
from cube import Digital_Cube
from env_RL import RL_environment
from RL import RL_brain
import argparse
import itertools
import cPickle as pickle
import matplotlib.pyplot as plt

"""
LOG

best parameters so far:
lr = .02, every million .9 it
epsilon = 1, every trial (1-epsilon*.0000002)

similar:
lr = .02, every million .99 it
epsilon = 1, every trial (1-epsilon*.0000002)

again similar:
lr adaptive
epsilon = 1, every trial (1-epsilon*.0000002)

even better: USING THIS
lr adaptive
epsilon = 1, every trial (1-epsilon*.000003)

even worse:
lr adaptive
epsilon = 1, every trial (1-epsilon*.00000009)

epsilon with .000005 and .000009 also work, even stronger
"""

class Main(object):
    """
    The main class, which can train and test the Q Learning Engine
    """

    def __init__(self, train):
        self.env = RL_environment()
        self.RL = None
        self.avg_reward = 0
        self.trials = 100000000
        self.trial_finished = False
        self.observation = None
        self.action = None
        self.reward = None
        self.observation2 = None
        self.test_interval = 1000
        self.mode = train


    def run(self):
        """
        determines whether to train or to test depending on runtime flag
        """

        if self.mode:
            self.train()
        else:
            self.test()

    def test(self):
        """
        Tests the supposedly already trained memory by loading it and then running
        trials on it without learning. Then displays results of tests
        """

        print "LOADING MEMORY"

        with open('/home/rooster/catkin_ws/src/irl/dino_arms/projects/Planning/Assembler/final_memory.txt', 'rb') as f:
            q_table = pickle.load(f)
        print "DONE"

        # creates a Q Learning Engine using an existing table from the file above
        self.RL = RL_brain(e_greedy=0, q_table=q_table)

        # runs 100 tests on the Q Learning Engine, and outputs accuracy
        print "------FIRST SHOWING TESTING ON 100\n"
        accuracy = 0
        for i in range(100):
            self.observation = self.env.reset()
            done_sequencing = False
            while not done_sequencing:
                self.action = self.RL.choose_action(str(self.observation))
                self.observation, self.reward, done_sequencing = self.env.step(self.action)
            accuracy += 0 if self.reward == -1 else 1
        print "TEST ACCURACY", accuracy, "%"

        # runs 10 more tests, but this time shows the target and the final state before it exited
        print "------NOW SHOWING VERBOSE ON 10\n"

        for i in range(10):
            self.observation = self.env.reset()
            target = self.env.target[:]
            done_sequencing = False
            sequence = []
            while not done_sequencing:
                self.action = self.RL.choose_action(str(self.observation))
                sequence.append(self.action)
                self.observation, self.reward, done_sequencing = self.env.step(self.action)

            print "-----CORRECT SEQUENCE-----" if self.reward != -1 else "MISSED IT"
            print "RL's SEQUENCE", sequence
            print "TARGET's LIST", target



    def train(self):
        """
        Trains the Q Learning Engine

        Creates environments, runs the Q Learning Engine through the environment,
        and then updates the engine based on the rewards it finds at the end, either by
        succeeding or failing
        """

        # making an empty Q Learning Engine
        self.RL = RL_brain()
        reward_list = []

        # run all the trials
        for i in range(self.trials):

            # reset the environment
            self.observation = self.env.reset()
            self.trial_finished = False
            if i%self.test_interval == 0:
                print "EPISODE", i

            # run through the environment, with the sequence described above
            while not self.trial_finished:

                self.action = self.RL.choose_action(str(self.observation))

                self.observation2, self.reward, self.trial_finished = self.env.step(self.action)

                self.RL.learn(str(self.observation), self.reward, str(self.observation2))

                self.observation = self.observation2


            self.RL.update_params()

            # every X trials, try it on 100 more tests, and output the accuracy
            # so we know how it's doing
            if i%self.test_interval == 0:
                self.avg_reward = 0
                for _ in range(100):
                    self.observation = self.env.reset()
                    self.trial_finished = False
                    while not self.trial_finished:
                        self.action = self.RL.choose_action(str(self.observation))
                        self.observation, self.reward, self.trial_finished = self.env.step(self.action)
                    self.avg_reward += self.reward
                self.avg_reward /= 100.0
                print "AVG. REWARD (per 100)", self.avg_reward, ""
                reward_list.append(self.avg_reward)

        print "FINISHED TRAINING"
        print "THERE ARE", len(self.RL.q_table), "TOTAL STATES"

        # at the end of the training, store the engine's memory
        with open('/home/rooster/catkin_ws/src/memory/final_memory.txt', 'wb') as f:
            pickle.dump(self.RL.q_table, f)

        print "MEMORY SAVED"

        # output a plot to show the entire training session's progress
        plt.plot(range(self.trials/self.test_interval), reward_list)
        plt.axis([0,self.trials/self.test_interval, -1, 1])
        plt.show()

if __name__ == "__main__":
    # argument flags for testing or training
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--train', action='store_true')
    args = parser.parse_args()

    main = Main(args.train)
    main.run()
