#!/usr/bin/env python
import rospy
import rospkg
import random
import math
# import st
import numpy as np
from std_msgs.msg import String
import time
import pickle, os, sys
from edwin.srv import *

class IdleBehaviors:
    def __init__(self):
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        rospy.init_node('idle', anonymous=True)

        rospy.Subscriber('/idle_cmd', String, self.control_callback, queue_size=10)
        rospy.Subscriber('/arm_status', String, self.status_callback, queue_size=10)

        self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=10)
        self.behav_pub = rospy.Publisher('/behaviors_cmd', String, queue_size=10)

        self.last_interaction = time.time()
        self.stop_idle_time = time.time()

        self.status = None


        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")

        self.routes = pickle.load(open(PACKAGE_PATH + '/params/routes.txt', 'rb'))
        self.idle_behaviors = pickle.load(open(PACKAGE_PATH + '/params/behaviors.txt', 'rb'))
        self.idle_behaviors = {key: value for key, value in self.idle_behaviors.items()
             if "idle" in key}
        print "idle is ready"


        self.idling = True
        self.idle_time = random.randint(5, 7)
        print "Starting idle node"


    def status_callback(self, data):
		print "arm status callback", data.data
		if data.data == "busy" or data.data == "error":
			self.status = 0
		elif data.data == "free":
			self.status = 1


    def check_completion(self):
		"""
		makes sure that actions run in order by waiting for response from service
		"""

		time.sleep(3)
		while self.status == 0:
			pass


    def request_cmd(self, cmd):
        rospy.wait_for_service('arm_cmd', timeout=15)
        cmd_fnc = rospy.ServiceProxy('arm_cmd', arm_cmd)
        print "I have requested the command"

        try:
            resp1 = cmd_fnc(cmd)
            print "command done"


        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            self.arm_status.publish('error')
            self.serv_prob = True


    def control_callback(self, data):
        print "IDLE CMD: ", data.data
        if "idle:stop" in data.data:
            self.idling = False
            self.stop_idle_time = time.time()
        elif "idle:go" in data.data:
            self.idling = True
            msg = random.choice(self.idle_behaviors.keys())
            print "PUBLISHING: ", msg
            self.behav_pub.publish(msg)
            self.check_completion()
            time.sleep(3)
        elif "idle:init" in data.data:
            rospack = rospkg.RosPack()
            PACKAGE_PATH = rospack.get_path("edwin")

            self.routes = pickle.load(open(PACKAGE_PATH + '/params/routes.txt', 'rb'))
            self.idle_behaviors = pickle.load(open(PACKAGE_PATH + '/params/behaviors.txt', 'rb'))
            self.idle_behaviors = {key: value for key, value in self.idle_behaviors.items() if "idle" in key}
            print "idle is ready"
            self.idling = True


    def reset_interval(self):
        self.last_interaction = time.time()


    def run(self):
        r = rospy.Rate(10)
        joints = ["H", "WR", "E", "WA", "BEHAV"]
        while not rospy.is_shutdown():
            if self.idling:
                if int(time.time() - self.last_interaction) > self.idle_time:
                    self.idle_time = random.randint(3, 7)
                    self.reset_interval()
                    print "IDLE"
                    joint = random.choice(joints)
                    if joint == "H":
                        msg = "data: rotate_hand:: " + str(random.randint(-900, 900))
                    elif joint == "WR":
                        msg = "data: rotate_wrist:: " + str(random.randint(-500, 2500))
                    elif joint == "E":
                        msg = "data: rotate_elbow:: " + str(random.randint(11000, 13000))
                    elif joint == "WA":
                        msg = "data: rotate_waist:: " + str(random.randint(4000, 6000))
                    elif joint == "BEHAV":
                        msg = random.choice(self.idle_behaviors.keys())

                    if joint == "BEHAV":
                        print "PUBLISHING: ", msg
                        self.behav_pub.publish(msg)
                        self.check_completion()
                    else:
                        print "PUBLISHING: ", msg
                        self.request_cmd(msg)

            # r.sleep()

if __name__ == '__main__':
    idle_eng = IdleBehaviors()
    idle_eng.run()
