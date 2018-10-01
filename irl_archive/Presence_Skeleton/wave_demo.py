#!/usr/bin/env python
import rospy
import math
import numpy as np
from std_msgs.msg import String, Int16
from edwin.msg import *
import time


class Waver:
    def __init__(self):
        rospy.init_node('edwin_presence', anonymous = True)
        rospy.Subscriber('wave_at_me', Int16, self.wave_callback, queue_size=10)

        self.behavior_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
        self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
        self.arm_pub.publish("data: set_speed:: 3000")


    def wave_callback(self, waves):

        if int(waves.data) == 1:
            print "I saw you wave! Hello!"
            msg = "data: R_nudge"
            self.behavior_pub.publish(msg)
            time.sleep(3)

    def run(self):
        print "running"
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    wave = Waver()
    time.sleep(2)
    wave.run()
