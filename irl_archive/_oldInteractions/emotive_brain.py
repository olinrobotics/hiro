#!/usr/bin/env python

"""
To run all the necessary components for the brain_eng

rosrun edwin arm_node.py
rosrun edwin arm_behaviors.py
rosrun edwin draw.py
rosrun edwin edwin_audio.py
rosrun edwin soundboard.py
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB1 _baud:=9600
"""

import rospy
import rospkg
import random
import math
import time
import numpy as np
import pickle, os, sys
from std_msgs.msg import String, Int16


from InteractiveDemos import TicTacToe as ttt

class EdwinBrain:
    def __init__(self):
        rospy.init_node('edwin_brain', anonymous=True)
        rospy.Subscriber('/edwin_imu', String, self.imu_callback, queue_size=1)
        rospy.Subscriber('/arm_status', Int16, self.arm_mvmt_callback, queue_size=1)
        rospy.Subscriber('/brain_move', String, self.move_callback, queue_size=2)

        self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=2)
        self.behav_pub = rospy.Publisher('/behaviors_cmd', String, queue_size=2)

        self.idling = True
        self.moving = False

        self.brain_state = 0 #negative = sad, positive = happy
        self.personality = 0.5 #negative = moody, positive = cheerful

        self.behaviors = {}
        self.cat_behaviors = {} #categorized behaviors
        self.create_behaviors()


    def create_behaviors(self):
        rospack = rospkg.RosPack()
        curr_dir = rospack.get_path("edwin")

        self.behaviors = pickle.load(open(curr_dir+'/params/'+str(self.personality)+'_behaviors.txt'))

        self.cat_behaviors['negative_emotions'] = ['angry', 'sad']
        self.cat_behaviors['happy_emotions'] = ['nod', 'butt_wiggle']
        self.cat_behaviors['greeting'] = ['greet', 'nudge', 'curiosity']
        self.cat_behaviors['pretentious']  = ['gloat']
        self.cat_behaviors['calm'] = ['sleep', 'nudge', 'nod']

        self.pat = False
        self.slap = False

        self.exit = False #should be catch all to exit all long running commands
        self.start_game = None

    def move_callback(self, data):
        s = random.randint(0, 10)
        if s > (self.brain_state * self.personality):
            behav_prefix = "happy_"
        else:
            behav_prefix = "sad_"

        self.behav_pub.publish(behav_prefix + data.msg)

    def arm_mvmt_callback(self, data):
        if data.data == 1:
            self.moving = True
        elif data.data == 0:
            self.moving = False

    def imu_callback(self, data):
        """
        IMU: no patted/slapped
        """
        state = data.data.replace("IMU: ", "")
        #print "STATE IS: ", state

        if self.moving == False:
            if state == "pat":
                self.brain_state += 5 #Edwin is happier
                self.pat = True
                self.slap = False

            elif state == "slap":
                self.brain_state -= 5 #Edwin is sad
                self.pat = False
                self.slap = True

    def update(self):
        if self.brain_state > 0:
            self.brain_state -= 0.5
        else:
            self.brain_state += 0.5

    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == '__main__':
    brain_eng = EdwinBrain()
    brain_eng.run()
