#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from edwin.srv import *
import time

class SrvTest(object):

    def __init__(self):
        rospy.init_node('tester', anonymous=True)
        self.status = -1
        self.pub2 = rospy.Publisher('/behaviors_cmd', String, queue_size=10)
        rospy.Subscriber('/arm_status', String, self.callback, queue_size=10)

    def callback(self, data):
        print "srv_test callback", data.data
        if data.data == "busy" or data.data == "error":
            self.status = 0
        elif data.data == "free":
            self.status = 1


    def run(self):


        r = rospy.Rate(10)
        print "starting up"
        time.sleep(3)
        while not rospy.is_shutdown():
            self.pub2.publish("heart")
            print "published heart"
            time.sleep(5)
            while self.status == 0:
                print "waiting on heart to finish"
            self.pub2.publish('bow')
            print "published bow"
            time.sleep(3)
            while self.status == 0:
                print "waiting on bow to finish"
            r.sleep()

# def add_two_ints_client(cmd):
#     rospy.wait_for_service('arm_cmd')
#     cmd_fnc = rospy.ServiceProxy('arm_cmd', arm_cmd)
#     print "i have requested"
#     try:
#         resp1 = cmd_fnc(cmd)
#         print "request finished"
#
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e

if __name__ == "__main__":
    node = SrvTest()
    node.run()
