#!/usr/bin/env python
import rospy
import rospkg

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import time
import math
import numpy as np
import os, sys

import cv2
import cv2.cv as cv

class FaceTracker:
    def __init__(self):
        rospy.init_node('face_tracking', anonymous=True)
        self.pub = rospy.Publisher('/face_location', String, queue_size=10)
        rospy.Subscriber("all_control", String, self.cmd_callback)

        self.bridge = CvBridge()
        rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)

        self.detect = True
        self.started_tracking = time.time()

        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        self.face_cascade = cv2.CascadeClassifier(PACKAGE_PATH + '/params/haarcascade_frontalface_default.xml')
        print self.face_cascade
        self.ideal_x = 640/2
        self.ideal_y = 480/2

    def img_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            h, w = self.frame.shape[:2]
        except CvBridgeError as e:
            print(e)

    def cmd_callback(self, data):
        print "GOT: ", data.data
        if "ft stop" in data.data:
            self.detect = False
        elif "ft go" in data.data:
            self.started_tracking = time.time()
            self.detect = True

    def face_tracking(self):
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        print "Faces found: ", len(faces)
        largest_face = (0, 0, 0, 0) #x, y, w, h
        for (x,y,w,h) in faces:
            cv2.rectangle(self.frame,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = self.frame[y:y+h, x:x+w]
            roi_color = self.frame[y:y+h, x:x+w]

            if w*h > largest_face[2]*largest_face[3]:
                largest_face = (x+(0.5*w), y+(0.5*h), w, h)

        #TODO: Publish a list of faces by their relative "largeness" so edwin
        #can track multiple people at once
        print "LF: ", largest_face
        loc_x = largest_face[0] - self.ideal_x
        loc_y = largest_face[1] - self.ideal_y

        self.pub.publish(str(loc_x)+":"+str(loc_y))
        # cv2.imshow("fr", self.frame)
        # cv2.waitKey(1)

    def run(self):
        r = rospy.Rate(10)
        time.sleep(2)
        while not rospy.is_shutdown():
            if self.detect:
                self.face_tracking()
                # if time.time() - self.started_tracking > 10:
                #     print "GOT BORED, STOPPED TRACKING"
                #     self.detect = False
            r.sleep()

if __name__ == '__main__':
    ft = FaceTracker()
    ft.run()
