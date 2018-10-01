#!/usr/bin/env python

"""
head_node.py publishes to head_cmd, a rosserial node located in the Arduino in Edwin's head
"""

import rospy
import math
import st
import numpy as np
from std_msgs.msg import String, Int8
import time

GRAB = 1
SPIT = 2

class HeadCommands:
    def __init__(self):
        rospy.init_node('robot_arm', anonymous=True)

        rospy.Subscriber('/head_cmd', String, self.head_callback, queue_size=1)
        self.head_pub = rospy.Publisher('head_serial', Int8, queue_size=10)

    def head_callback(self, cmdin):
        cmd = cmdin.data
        if cmd == "grip":
            self.head_pub.publish(GRAB)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    head_eng = HeadCommands()
    head_eng.run()
