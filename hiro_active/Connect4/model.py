import torch
import torch.nn as nn
import torch.nn.functional as fun
import numpy as np
import matplotlib

class DQN(nn.Module):

    def __init__(self, game):
        super(DQN, self).__init__()
        self.output_size = 64
        self.board_x, self.board_y = game.getBoardSize()
        self.input_size = torch.tensor([self.board_x, self.board_y])
        # x_image = torch.reshape(self.input_size, (-1,))
        self.action_size = game.getActionSize()
        self.n_layers = 3
        self.conv1 = nn.Conv2d(self.input_size[0], self.action_size, self.n_layers) #Will fix later, for some reason only one val of tensor (even tho it's 1-D)
        self.bn1 = nn.BatchNorm2d(self.action_size)
        self.head = nn.Linear(self.action_size, self.output_size)

    def forward(self, x):
        x = fun.relu(self.bn1(self.conv1(x)))
        # compress into 1D vector
        return self.head(x.view(x.size(0), -1))

class Loss(nn.Module):
    def __init__(self):
        super(Loss,self).__init()__
        self.loss_criterion = torch.nn.SmoothL1Loss()
        self.optimizer = torch.optim.Adam(model.parameters(), lr=lr)

    # def __init__(self):
    #     super(Loss, self).__init__()
    #
    # def forward(self, val, val2, policy, policy2):
    #     val_err = (val2 - val) ** 2
    #     policy_err = torch.sum((-policy2*torch.log(1e-8 + policy)), 1)
    #     total_err = torch.mean((val_err.view(-1) + policy_err))
    #     return total_err
