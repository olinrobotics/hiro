
import os
import sys
import time

import numpy as np
from tqdm import tqdm

import torch
from model import DQN as c4net


class NeuralNet():
    """
    This class specifies the base NeuralNet class. The neural
    network does not consider the current player, and instead only deals with
    the canonical form of the board.
    See othello/NNet.py for an example implementation.
    """

    def __init__(self, game):
        self.nnet = c4net(game, 64)
        self.board_x, self.board_y = game.getBoardSize()
        self.action_size = game.getActionSize()
        self.saver = None

    def save_checkpoint(self, folder, filename):
        """
        Saves the current neural network (with its parameters) in
        folder/filename
        """
        filepath = os.path.join(folder, filename)
        if not os.path.exists(folder):
            print("Checkpoint Directory does not exist! Making directory {}".format(folder))
            os.mkdir(folder)
        else:
            print("Checkpoint Directory exists! ")
        if self.saver == None:
            params = [data_width, data_width, state_size, n_layers]
            self.saver = torch.save({"state_dict": c4rl.state_dict(), "params": params}, filepath)

    def load_checkpoint(self, folder, filename):
        """
        Loads parameters of the neural network from folder/filename
        """
        filepath = os.path.join(folder, filename)
        if not os.path.exists(filepath + '.meta'):
            raise ("No model in path {}".format(filepath))
        c4rl.load_state_dict(torch.load(filepath))
        c4rl.eval()
