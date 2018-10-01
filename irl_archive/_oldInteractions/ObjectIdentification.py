#!/usr/bin/env python
import rospy
import cv2
import cv2.cv as cv
import numpy as np
import random
import time
import math
import operator
import itertools

from std_msgs.msg import String
from sensor_msgs.msg import Image
from edwin.msg import Edwin_Shape
from cv_bridge import CvBridge, CvBridgeError

"""
AutoPlay is a demo in which Edwin interacts with objects in his field of view. This is
a non-interactive demo that is halted once a human is detected in the scene.

There are multiple interaction types in this demo, and it concludes once a full run-through
of one has been down.
"""
class Game:
	def __init__(self):
		self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
		self.behav_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)

	def img_callback(self, data):
		try:
			self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

	def run(self):
        print "Starting ObjectIdentification demo"

        running = True
		while running:
            pass

        print "Finished with ObjectIdentification :)"

if __name__ == '__main__':
	rospy.init_node('oi_gamemaster', anonymous = True)
	gm = Game()
	gm.run()
