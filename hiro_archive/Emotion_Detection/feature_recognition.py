#!/usr/bin/env python
import rospy
import rospkg
import time

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

import cv2

class FeatureRecognition:
    def __init__(self):
        rospy.init_node('feature_recognition', anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)
        self.pub = rospy.Publisher('/contour_size', String, queue_size=10)
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        self.detect = True
        self.lower_blue = np.array([28,70,123])
        self.upper_blue = np.array([50,255,255])

    def img_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def output_image(self):
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        max_size=0
        largest_index=0
        for x in range(0,len(contours)):
            if contours[x].size>max_size:
                largest_index=x
                max_size=contours[x].size
        res = cv2.drawContours(self.frame,contours,largest_index,(0,255,0),3)
        self.pub.publish(str(max_size))
        cv2.imshow('image', self.frame)
        cv2.waitKey(2)

    def run(self):
        r = rospy.Rate(10)
        time.sleep(2)
        while not rospy.is_shutdown():
            if self.detect:
                self.output_image()
            r.sleep()

if __name__ == '__main__':
    fr = FeatureRecognition()
    fr.run()
