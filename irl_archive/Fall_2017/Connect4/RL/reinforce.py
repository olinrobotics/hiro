#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import pandas as pd
import random
import time
from std_msgs.msg import String, Int16
import cPickle as pickle
import argparse
from game_board import C4Board
from n_rl_brain import QLearningTable
from Queue import *
from minimax import Minimax


class AI_Player(object):

    def __init__(self, n_actions, memory, reset, player_type):
        self.player_type = player_type
        self.observation = None
        self.observation_ = None
        self.action = None
        self.reward = None
        self.memory = memory
        self.lut = QLearningTable(actions=list(range(n_actions)),
                                q_table= None if reset else self.load_q_table())


    def load_q_table(self):
        with open(self.memory, 'rb') as f:
            return pickle.load(f)

    def store_memory(self):
        with open(self.memory, 'wb') as f:
            pickle.dump(self.lut.q_table, f)

    def set_observation(self, observation):
        self.observation = observation

    def set_observation_(self, observation_):
        self.observation_ = observation_

    def set_reward(self, reward):
        self.reward = reward

    def choose_action(self):
        self.action = self.lut.choose_action(str(self.observation))

    def learn(self):
        self.lut.learn(str(self.observation), self.action, self.reward, str(self.observation_))


class Reinforce(object):

    def __init__(self, train, reset, render):
        self.board = C4Board(render)
        self.RL1, self.RL2 = self.initialize_AI(reset)
        self.test = train


    def initialize_AI(self, reset):
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("irl")

        player_path1 = PACKAGE_PATH + "/projects/Connect4/memory/player1.txt"
        player_path2 = PACKAGE_PATH + "/projects/Connect4/memory/player2.txt"

        player_1 = AI_Player(self.board.n_actions, player_path1, reset, 1)
        player_2 = AI_Player(self.board.n_actions, player_path2, reset, 2)

        return player_1, player_2


    def run(self):
        if self.test:
            self.train()
        else:
            self.play()


    def play(self):

        player_type = 1
        ai = self.RL1 if player_type == 1 else self.RL2
        ai.set_observation(self.board.reset())

        # if ai.player_type == 1:
        print " I WILL GO FIRST"
        ai.choose_action()
        observation_, _, _, done = self.board.step(ai.action, 1)

        while True:
            player_action = None
            while player_action not in range(1,8):
                player_action = int(raw_input("YOUR TURN, WHAT'S YOUR MOVE? (1-7) "))
                print "INVALID MOVE" if player_action not in range(1,8) else ""

            observation_, _, _, done = self.board.step(player_action-1, 2)
            ai.set_observation(observation_)

            # break if I win
            if done:
                print "\n"*2
                print "HUMAN WINS"
                break

            ai.choose_action()
            observation_, _, _, done = self.board.step(ai.action, 1)

            # break if ai wins
            if done:
                print "\n"*2
                print "AI WINS"
                break

        print "PLAYING OVER"


    # def train(self):
    #     for episode in range(10000):
    #         # initial observation
    #         self.RL1.set_observation(self.board.reset())
    #         print "EPISODE", episode
    #         while True:
    #
    #             # RL choose action based on observation
    #             self.RL1.choose_action()
    #
    #             # RL take action and get next observation and reward
    #             observation1_, reward1, reward2, done = self.board.step(self.RL1.action, 1)
    #
    #             self.RL1.set_observation_(observation1_)
    #             self.RL1.set_reward(reward1)
    #             self.RL2.set_reward(reward2)
    #
    #             if done:
    #                 self.RL1.learn()
    #                 self.RL2.learn()
    #                 self.scoreboard += 1 if self.RL1.reward == 1 else 0
    #                 print self.scoreboard/float(episode)
    #                 break
    #
    #             else:
    #                 if self.RL2.action:
    #                     self.RL2.learn()
    #
    #             self.RL2.set_observation(self.RL1.observation_)
    #
    #             # RL choose action based on observation
    #             self.RL2.choose_action()
    #
    #             # RL take action and get next observation and reward
    #             observation2_, reward1, reward2, done = self.board.step(self.RL2.action, 2)
    #
    #             self.RL2.set_observation_(observation2_)
    #             self.RL2.set_reward(reward2)
    #             self.RL1.set_reward(reward1)
    #
    #             if done:
    #                 self.RL1.learn()
    #                 self.RL2.learn()
    #                 break
    #             else:
    #                 self.RL1.learn()
    #
    #             self.RL1.set_observation(self.RL2.observation_)
    #
    #
    #     print "TRAINING OVER"
    #     self.RL1.store_memory()
    #     self.RL2.store_memory()
    #     print "MEMORY STORED, SESSION FINISHED"


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--train', action='store_true')
    parser.add_argument('-r', '--reset', action='store_true')
    parser.add_argument('-v', '--render', action='store_true')
    args = parser.parse_args()

    connect = Reinforce(args.train, args.reset, args.render)
    connect.run()
