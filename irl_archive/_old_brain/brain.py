#!/usr/bin/env python

"""
This is the brain for Edwin Spring 2017

Integrates SimonSays and Homework Solver into a cohesive demo

To run all the necessary components for the brain.py

Please run:

rosrun edwin idle.py

For SimonSays:
-all skeleton stuff (roslaunch, rviz, skeleton.py, presence_detection, minimal launch)
-tts and stt simon

For Homework Solver:
-arm_write.py
-roslaunch usb_cam usb_camera.launch
-handwriting_recognition.py

How it works:
1. While nothing is happening, be idle and do idle things
2. When a person is detected with skeleton, presence_detection kicks in and finds the person
the person will be followed for this duration
3. The person will enter visual menu, calibrate, and then select a mode
4. After choosing a demo, the demo will execute and the person will do the demo
5. The demo is over, and now Edwin will re-enter Step 1

"""

import rospy
import rospkg
import random
import math
import time
import numpy as np
import pickle, os, sys
from std_msgs.msg import String, Int16


from Interactions.SimonSays import EdwinSimon, EdwinPlayer
from sight.math_interp import Calculator
from sight.handwriting_recognition import HandwritingRecognition


class EdwinBrain:
    def __init__(self, manual):
        rospy.init_node('edwin_brain', anonymous=True)

        self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=2)
        self.behav_pub = rospy.Publisher('/behaviors_cmd', String, queue_size=2)
        self.idle_pub = rospy.Publisher('/idle_cmd', String, queue_size=10)
        self.presence_pub = rospy.Publisher('/presence_cmd', String, queue_size=10)

        rospy.Subscriber('/tracking', Int16, self.tracking_callback, queue_size=10)
        rospy.Subscriber('/detected', String, self.detection_callback, queue_size=10)


        self.manual = manual

        #visualmenu stuff
        self.menu_choice = None

        #presence stuff
        self.detected = False
        self.tracking = 0


        #idle starts now
        self.idle_pub.publish("idle:init")

        time.sleep(1)
        print "edwin brain is running"


    def tracking_callback(self, data):
        """
        Receives tracking data on whether a person is detected
        """

        self.tracking = data.data


    def detection_callback(self, data):
        """
        Receives detection data, whether something was detected or not
        """

        if data.data == "found":
            self.detected = True
        elif data.data == "nothing":
            self.detected = False

    def demo(self):
        """
        Main loop that does the demo
        """
        if self.manual:
            self.menu_choice = raw_input("What game would you like to play?\n")
        else:
            self.menu_choice = self.menu.run()
        # self.idle_pub.publish("idle:stop")
        self.presence_pub.publish('stop')
        time.sleep(1)
        if self.menu_choice == "simon":
            game = EdwinSimon(rospy)
            game.run()
        elif self.menu_choice == "player":
            difficulty = raw_input("How good should Edwin be?\n")
            game = EdwinPlayer(difficulty, rospy)
            game.run()
        elif self.menu_choice == "homework":
            game = Calculator(rospy)
            game.run()

        time.sleep(3)
        # self.idle_pub.publish("idle:go")


    def run(self):
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.detected:
                self.demo()
                print "Demo done"
                while self.tracking == 1:
                    pass
                print "User has left"
                self.detected = False
                self.presence_pub.publish('start')

            r.sleep()





if __name__ == '__main__':
    manual = True
    TheBrain = EdwinBrain(manual)
    TheBrain.run()
