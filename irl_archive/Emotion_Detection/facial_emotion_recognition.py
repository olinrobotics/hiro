#!/usr/bin/env python
import rospkg
import rospy

import cv2
import sys
import logging as log
import datetime as dt
import time

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class FaceDetect:

    def __init__(self, init=False):
        #initializes ros node for face detect, pubilishes to face location
        self.init = init
        if not self.init:
            rospy.init_node('face_detect', anonymous=True)
            self.pub = rospy.Publisher('/smile_detected', String, queue_size = 10)
            #detection boolean
            self.detect = True
        else:
            self.pub = rospy.Publisher('/behaviors_cmd', String, queue_size = 10)
            self.image_pub = rospy.Publisher('/edwin_image', Image, queue_size = 10)

        self.running = True

        #definines cascade path
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        self.face_cascade = cv2.CascadeClassifier(PACKAGE_PATH + '/params/haarcascade_frontalface_alt.xml')
        self.smile_cascade = cv2.CascadeClassifier(PACKAGE_PATH + '/params/haarcascade_smile.xml')

        #CvBridge to usb_cam, subscribes to usb cam
        self.bridge = CvBridge()
        rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)

        #initializes frame
        self.frame = None
        self.smile_counter = 0
        print "FaceDetect is running"

    #converts ros message to numpy
    def img_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            h, w = self.frame.shape[:2]
        except CvBridgeError as e:
            print(e)

    def publish_image(self, data):
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(data, "bgr8"))
        except CvBridgeError as e:
            print(e)

    #face_detect method
    def face_detect(self):
        if self.frame == None:
            return

        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.2,
            minNeighbors=5,
            minSize=(20, 20),
            flags=cv2.cv.CV_HAAR_SCALE_IMAGE
        )

        face = None
        #draws rectangle around face

        #biggest face
        biggestArea = 0;
        smile_msg = 'False'
        msg = ''
        for (x, y, w, h) in faces:
            cv2.rectangle(self.frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

            #crops face from image
            face = gray[y:y+h,x:x+w]

            smile = self.smile_cascade.detectMultiScale(
                face,
                scaleFactor= 1.7,
                minNeighbors=22,
                minSize=(25, 25),
                flags=cv2.cv.CV_HAAR_SCALE_IMAGE
                )

            #find biggest face
            area = w*h;
            center_x = x + w/2
            center_y = y + h/2

            if len(smile) > 0:
                smile_msg = 'True'
                msg = 'True'
                
            for (x, y, w, h) in smile:
                print "Found", len(smile), "smiles!"
                self.running = False

                cv2.rectangle(self.frame, (x, y), (x+w, y+h), (255, 0, 0), 1)


        if not self.init:
            if msg != '':
                self.pub.publish(msg)

            # #displays resulting frame
            cv2.imshow('Video', self.frame)
            if face != None:
                cv2.imshow('Face', face)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.detect = False
        else:
            if msg != '':
                self.pub.publish("laugh") #Arm behavior's pattern for smile response
                print "SMILE :)"
            else:
                if int(time.time() - self.init_time) > 10:
                    print "no smile found"
                    self.pub.publish("pout")
                    self.running = False
            self.publish_image(self.frame)

    def demo_run(self):
        self.init_time = time.time()

        while self.running:
            self.face_detect()

    def run(self):
        while not rospy.is_shutdown():
            if self.detect:
                self.face_detect()

if __name__ == '__main__':
    fd = FaceDetect()
    fd.run()
