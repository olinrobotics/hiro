#!/usr/bin/env python

import rospy
import numpy as np
import pandas as pd
import random
import time
from std_msgs.msg import String, Int16
import pickle
import argparse
from game_board import C4Board
from rl_brain import QLearningTable
from Queue import *
from joblib import Parallel, delayed


def massive_function():
    thing = []
    for i in range(10000):
        thing.append(i)
        for j in range(42):
            thing.append(j)


    return thing


if __name__== "__main__":
    for i in range(10):
        print i
        thing2 = Parallel(n_jobs=1, verbose=1)(delayed(massive_function)() for i in range(40))
        for item in thing2:
            item = 1
    print "done"
