#!/usr/bin/env python

import os
import argparse
import time
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

"""
Author: Benjamin Ziemann
Purpose: Monitors that state of LEDs. When one turns on, a message is sent out
    to say what color it was
Dependencies:
    OpenCV
    Numpy
    ROS
To run:
    "python light"
"""

class lightDetector():

    def __init__(self):
        #How long to wait to consider something not a blip
        self.WAIT_TIME = .5

        #pull images from edwin's right camera
        self.bridge = CvBridge()
        self.obtained_frame = False
        self.frame = None
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.make_frame,queue_size = 10)
        #Storage setup
        self.prevStates =[1,1,1]
        self.currStates = [1,1,1]

        #video capture setup
        self.cap = cv2.VideoCapture(0)
        self.kernel = np.ones((5,5),np.uint8)

        #ROS Publisher
        try:
            self.pub = rospy.Publisher('chatter', String, queue_size=10)
            rospy.init_node('talker', anonymous=True)
            self.rate = rospy.Rate(10) # 10hz
        except rospy.ROSInterruptException:
            pass

        # Setup SimpleBlobDetector parameters.
        self.params = cv2.SimpleBlobDetector_Params()
        self.params.minThreshold = 10
        self.params.maxThreshold = 200
        self.params.filterByArea = True
        self.params.minArea = 150
        self.params.maxArea = 100000000
        self.params.filterByCircularity = False
        self.params.filterByConvexity = False
        self.params.filterByInertia = False

        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            self.detector = cv2.SimpleBlobDetector(self.params)
        else :
            self.detector = cv2.SimpleBlobDetector_create(self.params)

        #Time counter
        self.prevTime = time.time()
        self.currTime = time.time()

    def make_frame(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.obtained_frame = True
        except CvBridgeError, e:
            self.obtained_frame = False
            print e

    def blobCapture(self,lower_h, upper_h, lower_s, upper_s, lower_v, upper_v,frame):
        """
        Takes in upper and lower HSV values and uses it to pick out blobs from
        the webcam image
        """
        blur = cv2.GaussianBlur(frame,(5,5),0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        #Upper and lower HSV for masking
        lower_hsv = np.array([lower_h,lower_s,lower_v])
        upper_hsv = np.array([upper_h,upper_s,upper_v])
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)


        # Filter image
        blur = cv2.GaussianBlur(frame,(5,5),0)
        masked = cv2.bitwise_and(blur,blur, mask= mask)
        greyed = cv2.cvtColor(masked,cv2.COLOR_BGR2GRAY)
        ret,threshed = cv2.threshold(greyed,10,255,cv2.THRESH_BINARY_INV)
        closed = cv2.morphologyEx(threshed, cv2.MORPH_OPEN, self.kernel)

        #Find blobs
        resBlobs = self.detector.detect(closed)

        return resBlobs

    def run(self):
        """
        Determines whether a light turned on and sends a message if so
        """
        while(True):
            frame = self.frame
            #Get blob list of different colors detected
            redBlobs = self.blobCapture(-15,15,140,255,50,255,frame)   #Red
            #yellowBlobs = self.blobCapture(15,45,50,255,70,255,frame)   #Yellow
            greenBlobs = self.blobCapture(50,90,90,165,150,210,frame)       #Green
            blueBlobs = self.blobCapture(105,135,50,255,50,255,frame)   #Blue


            #compare current state to last known states
            self.currStates = [len(redBlobs),len(greenBlobs),len(blueBlobs)]

            flagPressed = False
            flagTimed = False

            self.currTime = time.time()
            #Checks to see if there were blips
            #Initial difference
            print(self.currStates)
            print(self.prevStates)
            if(self.currStates != self.prevStates):

                flagTimed = True
                if(flagTimed) and (self.currTime-self.prevTime > self.WAIT_TIME): #if enough time has passed

                    if not (self.currStates[0]):
                        mess = "Red on"
                        flagPressed = True

                    elif not (self.currStates[1]):
                        mess = "Green on"
                        flagPressed = True
                    elif not (self.currStates[2]):
                        mess = "Blue on"
                        flagPressed = True

                    if flagPressed:
                        print(mess)
                        flagPressed = False
                        flagTimed = False
                        self.pub.publish(mess)
                        time.sleep(0.01)

                elif (self.currTime-self.prevTime < self.WAIT_TIME): #Continue looping if not enough time
                    print("wait state")
                    continue
                else:   #Too much time has passed, it was a blip
                    flagTimed = False
                    self.prevStates = self.currStates
                    self.prevTime = self.currTime

            #No difference detected
            else:
                self.prevStates = self.currStates
                self.prevTime = self.currTime


            #Uncomment to see visuals

            drawBlobs1 = cv2.drawKeypoints(frame, redBlobs, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            drawBlobs2 = cv2.drawKeypoints(drawBlobs1, greenBlobs, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            drawBlobs3 = cv2.drawKeypoints(drawBlobs2, blueBlobs, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            #drawBlobs = cv2.drawKeypoints(drawBlobs3, yellowBlobs, np.array([]), (0,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            #res1 = cv2.addWeighted(drawBlobs,0.5,drawBlobs1,0.5,0)
            res2 = cv2.addWeighted(drawBlobs2,0.5,drawBlobs3,0.5,0)
            result = cv2.addWeighted(drawBlobs1,0.5, res2,0.5,0)

            cv2.imshow('blur', result)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    ld1 = lightDetector()
ld1.run()
