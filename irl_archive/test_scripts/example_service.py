#!/usr/bin/env python
import rospy
import math
import numpy as np
from std_msgs.msg import String, Int16
import time
from edwin.srv import arm_cmd

class ArmCommands:
    def __init__(self):
        rospy.init_node('robot_arm', anonymous=True)
        s = rospy.Service('arm_cmd', arm_cmd, self.arm_callback)
        print "Service ONLINE"


    def arm_callback(self, cmdin):
        print cmdin.cmd
        time.sleep(5)
        return ["I have completed the command"]

    def run(self):
        print "Service is ready to go"
        rospy.spin()
        # r = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     r.sleep()

if __name__ == "__main__":
    arm_eng = ArmCommands()
    arm_eng.run()
