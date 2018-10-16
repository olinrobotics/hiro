#!/usr/bin/env python

"""
Calibrates the correct depth of the pen for use in arm_draw
"""

import rospy
import random
import math
import time
import numpy as np
from std_msgs.msg import String

class DepthCalibrator:
    def __init__(self):
        rospy.init_node('edwin_depth', anonymous=True)

        self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=2)
        self.idle_pub = rospy.Publisher('/idle_cmd', String, queue_size=2)
        print "Starting depth calibrator"

        self.final_depth = 0

    def run(self):
        self.idle_pub("stop_idle")
        print "Calibrating at center of paper"
        msg = "data: move_to:: 0, 4000, -500, 0"
        self.arm_pub(msg)

        for i in range(0, 350, 5):
            msg = "data: move_to:: 0, 4000, " + str(i) + ", 0"
            time.sleep(1)
            self.arm_pub(msg)

if __name__ == '__main__':
    calib = DepthCalibrator()
    calib.run()
