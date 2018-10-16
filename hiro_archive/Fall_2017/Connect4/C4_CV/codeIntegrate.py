'''test code for integrating c4imagproc.py into larger code'''

import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt
import time

from c4imageproc.py import DetectConnectFour

detect = DetectConnectFour(3,2)

rospy.init_node('boardCaller')
pub = rospy.Subscriber('opponent_move', int16, queue_size=10)
