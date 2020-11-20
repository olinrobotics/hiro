import torch
import torch.nn as nn
import torch.nn.functional as fun
import numpy as np
import matplotlib


class CNN(nn.Module):
    def __init__(self):
        super(CNN, self).__init__()
        self.conv1 = nn.Conv2d(3, 32, 3, stride=1, padding=1)
        self.bn1 = nn.BatchNorm2d(32)

    def forward(self, s):
        s = s.view(-1, 3, 6, 7)  # compress into 1D vector, channels, board x, board y
        s = self.conv1(s)
        s = self.bn1(s)
        s = fun.relu(s)
        return s

class RNN(nn.Module):
    def __init__(self, input_size=32, output_size=32, stride=1, downsample=None):
        super(RNN, self).__init__()
        self.conv1 = nn.Conv2d(input_size, output_size, kernel_size=3, stride=stride, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(input_size)

    def forward(self, x):
        res = x
        out = self.conv1(x)
        out = self.bn1(out)
        out = fun.relu(out)
        out += res
        out = fun.relu(out)
        return out

# Need a block to combine the output of the two nets
    
class Connect(nn.Module):
    def __init__(self):
        super(Connect, self).__init__()
        self.conv = CNN()
        blocks = 15
        for block in range(blocks):
            setattr(self, "res_%i" % block,RNN())
        # Put connected output block here self.output
    
    def forward(self,s):
        s = self.conv(s)
        for block in range(blocks):
            s = getattr(self, "res_%i" % block)(s)
        # Put connected output block here s =
        return s

# Include loss