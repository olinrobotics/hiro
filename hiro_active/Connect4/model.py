import torch
import torch.nn as nn
import torch.nn.functional as fun
import numpy as np
import matplotlib

class DQN(nn.Module):

    def __init__(self, input_size, output_size, state_size, n_layers):
        super(DQN, self).__init__()
        self.conv1 = nn.Conv2d(input_size, state_size, n_layers)
        self.bn1 = nn.BatchNorm2d(state_size)
        self.head = nn.Linear(state_size, output_size)

    def forward(self, x):
        x = fun.relu(self.bn1(self.conv1(x)))
        # compress into 1D vector
        return self.head(x.view(x.size(0), -1))

class Loss(nn.Module):
    def __init__(self):
        super(Loss, self).__init__()

    def forward(self, val, val2, policy, policy2):
        val_err = (val2 - val) ** 2
        policy_err = torch.sum((-policy2*torch.log(1e-8 + policy)), 1)
        total_err = torch.mean((val_err.view(-1) + policy_err))
        return total_err
