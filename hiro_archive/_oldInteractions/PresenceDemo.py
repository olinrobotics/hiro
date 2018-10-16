#!/usr/bin/env python
import rospy
import cv2
import cv2.cv as cv
import numpy as np
import random
import time
import math

from std_msgs.msg import String

from sight import presence_detection
from sight import facial_emotion_recognition

"""
PresenceDemo is a demo in which Edwin interacts with people that he sees using the Kinect.
This demo ends once someone waves at Edwin and smiles.
"""

class Game:
	def __init__(self):
		self.presence_demo = presence_detection.Presence(init=True)
		self.face_detect = facial_emotion_recognition.FaceDetect(init=True)

		self.pub = rospy.Publisher('all_control', String, queue_size=1)

	def run(self):
		self.pub.publish("singlecontrol:start;sceneanalysis:start")
		print "Starting PresenceDemo"
		self.presence_demo.run() #ends when someone waves at Edwin
		self.pub.publish("singlecontrol:stop;sceneanalysis:stop")
		print "Waiting for smile"
		self.face_detect.demo_run()
		print "Finished with PresenceDemo :)"

if __name__ == '__main__':
	rospy.init_node('pd_gamemaster', anonymous = True)
	gm = Game()
	gm.run()
