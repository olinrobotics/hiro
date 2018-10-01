#!/usr/bin/env python

import numpy as np
import pandas as pd
import random
import time
import cPickle as pickle
import json
import argparse
from game_board import C4Board
from n_rl_brain import QLearningTable
from Queue import *
# from joblib import Parallel, delayed


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


    def reset(self):
        self.observation = None
        self.observation_ = None
        self.action = None
        self.reward = None

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
        self.action = self.lut.choose_action(self.observation)

    def learn(self):
        self.lut.learn(self.observation, self.action, self.reward, self.observation_)


class Reinforce(object):

    def __init__(self, train, reset, render):
        self.env = C4Board(render)
        self.RL1 = AI_Player(self.env.n_actions, '/home/kzhang/irl/dino_arms/projects/Connect4/memory/player1.txt', reset, 1)
        self.RL2 = AI_Player(self.env.n_actions, '/home/kzhang/irl/dino_arms/projects/Connect4/memory/player2.txt', reset, 2)
        # self.RL1 = AI_Player(self.env.n_actions, '/Users/Zhangster/Documents/Kevin/Organizations/Edwin/irl/dino_arms/projects/Connect4/memory/player1.txt', reset, 1)
        # self.RL2 = AI_Player(self.env.n_actions, '/Users/Zhangster/Documents/Kevin/Organizations/Edwin/irl/dino_arms/projects/Connect4/memory/player2.txt', reset, 2)


    def run(self):
        for episode in range(10000000):
            print episode
            self.play_game()
            # self.batch_learn(game_aftermath)
            self.RL1.lut.update_params()
            self.RL2.lut.update_params()

        print "TRAINING OVER"
        print len(self.RL1.lut.q_table)
        self.RL1.store_memory()
        self.RL2.store_memory()
        print "MEMORY STORED, SESSION FINISHED"


    # def batch_learn(self, session):
    #     for record in session:
    #         for game in record:
    #             for r_combo in game:
    #               if r_combo[0] == 1:
    #                   self.RL1.lut.learn(r_combo[1], r_combo[2], r_combo[3], r_combo[4])
    #               else:
    #                   self.RL2.lut.learn(r_combo[1], r_combo[2], r_combo[3], r_combo[4])
    #

    def play_game(self):
        # session = []
        # for i in range(50):
        #     game_record = []
        self.RL1.reset()
        self.RL2.reset()
        # initial observation
        self.RL1.set_observation(self.env.reset())
        while True:

            # RL choose action based on observation
            self.RL1.choose_action()

            # RL take action and get next observation and reward
            observation1_, reward1, reward2, done = self.env.step(self.RL1.action, 1)
            self.RL1.set_observation_(observation1_)
            self.RL1.set_reward(reward1)
            self.RL2.set_reward(reward2)

            if done:
                # game_record.append((1, RL1.observation, RL1.action, RL1.reward, RL1.observation_))
                # game_record.append((2, RL2.observation, RL2.action, RL2.reward, RL2.observation_))
                self.RL1.learn()
                self.RL2.learn()
                break

            else:
                if self.RL2.action:
                    # game_record.append((2, RL2.observation, RL2.action, RL2.reward, RL2.observation_))
                    self.RL2.learn()

            self.RL2.set_observation(self.RL1.observation_)

            # RL choose action based on observation
            self.RL2.choose_action()

            # RL take action and get next observation and reward
            observation2_, reward1, reward2, done = self.env.step(self.RL2.action, 2)

            self.RL2.set_observation_(observation2_)
            self.RL2.set_reward(reward2)
            self.RL1.set_reward(reward1)

            if done:
                # game_record.append((1, RL1.observation, RL1.action, RL1.reward, RL1.observation_))
                # game_record.append((2, RL2.observation, RL2.action, RL2.reward, RL2.observation_))
                self.RL1.learn()
                self.RL2.learn()
                break
            else:
                # game_record.append((1, RL1.observation, RL1.action, RL1.reward, RL1.observation_))
                self.RL1.learn()

            self.RL1.set_observation(self.RL2.observation_)


            # session.append(game_record)
        # return session



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--train', action='store_true')
    parser.add_argument('-r', '--reset', action='store_true')
    parser.add_argument('-v', '--render', action='store_true')
    args = parser.parse_args()

    connect = Reinforce(args.train, args.reset, args.render)
    connect.run()
