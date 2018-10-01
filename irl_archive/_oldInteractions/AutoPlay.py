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
		self.draw_pub = rospy.Publisher('draw_cmd', Edwin_Shape, queue_size=10)
		self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
		self.behav_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)

	def img_callback(self, data):
		try:
			self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

    def wait_for_hand(self):
        gray = cv2.cvtColor(self.frame,cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        # noise removal
        kernel = np.ones((5, 5),np.uint8)
        opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)

        # sure background area
        sure_bg = cv2.dilate(opening,kernel,iterations=3)

        contours, hierarchy = cv2.findContours(sure_bg,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(self.frame, contours, -1, (0,255,0), 3)
        # Find the index of the largest contour
        areas = [cv2.contourArea(c) for c in contours]

        if len(areas) == 0:
            areas = [0]

        #TODO: Fix so this isn't necessary.
        #THRESH_OTSU automatically finds the thresh level, so if nothing is in the field
        #the thresh level goes extremely high. To account for this, perhaps make do a
        #two layer thresh with color?
        if max(areas) > 100000:
            return False
        elif max(areas) > 10000:
            print "LEN: ", len(areas)
            print "AREA: ", max(areas)
            print "FOUND HAND"
            return True
        else:
            return False

	def field_scan(self):
		time.sleep(5)

		cv2.imshow("img", self.frame)
		c = cv2.waitKey(1)

	def run(self):
        print "Starting AutoPlay"

        running = True
		while running:


		cv2.destroyAllWindows()
		print "Finished with AutoPlay :)"

if __name__ == '__main__':
	rospy.init_node('ss_gamemaster', anonymous = True)
	gm = Game()
	gm.run()
