#!/usr/bin/env python

import sys
import rospy
from edwin.srv import *

def add_two_ints_client(cmd):
    rospy.wait_for_service('arm_cmd')
    cmd_fnc = rospy.ServiceProxy('arm_cmd', arm_cmd)
    print "i have requested"

    try:
        resp1 = cmd_fnc(cmd)
        print "request complete"
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    index = 0
    while True:
        index += 1
        print index
        print add_two_ints_client(str("data: rotate_waist:: " + str(0)))
        print add_two_ints_client(str("data: rotate_waist:: " + str(5000)))
        print " "
