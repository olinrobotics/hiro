#!/usr/bin/env python

import rospkg
import rospy

import sys
import os
import cv2
import dlib
import glob
from skimage import io

from time import sleep
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class FaceDetect:

    def __init__(self):
        #initializes frame
        self.frame = None

        #initializes ros node for face detect, pubilishes to face location
        rospy.init_node('face_detect', anonymous=True)
        self.pub = rospy.Publisher('/face_location', String, queue_size=10)

        #definines file paths
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        self.predictor_path = PACKAGE_PATH + '/params/shape_predictor_68_face_landmarks.dat'
        self.faces_folder_path = PACKAGE_PATH + '/params'

        #def of attributes
        self.detect = True
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor(self.predictor_path)
        self.window = dlib.image_window()

        #CvBridge to usb_cam, subscribes to usb cam
        self.bridge = CvBridge()
        rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)

    #converts ros message to numpy
    def img_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            h, w = self.frame.shape[:2]
        except CvBridgeError as e:
            print(e)

    def get_landmarks(self, clahe_image):
        #code from http://www.paulvangent.com/2016/08/05/emotion-recognition-using-facial-landmarks/
        #detect faces in the image
        detections = self.detector(clahe_image, 1)

        #for each detected face
        for k,d in enumerate(detections):

            #Get coordinates
            shape = self.predictor(clahe_image, d)

            xlist = []
            ylist = []
            landmarks = []

            #For each point, draw a red circle with thickness2 on the original frame
            for i in range(1,68): #There are 68 landmark points on each face
                cv2.circle(self.frame, (shape.part(i).x, shape.part(i).y), 1, (0,0,255), thickness=2)
                #cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]])
                org = (shape.part(i).x, shape.part(i).y)
                fontFace = cv2.FONT_HERSHEY_PLAIN
                cv2.putText(self.frame, str(i), org, fontFace, 1,  (255,225,225) , 1, 8)

                xlist.append(float(shape.part(i).x))
                ylist.append(float(shape.part(i).y))

            for x, y in zip(xlist, ylist): #Store all landmarks in one list in the format x1,y1,x2,y2,etc.
                landmarks.append(x)
                landmarks.append(y)
        if len(detections) > 0:
            return landmarks
        else: #If no faces are detected, return error message to other function to handle
            landmarks = "error"
            return landmarks

    def face_detect(self):
        if self.frame == None:
            return

        #converts feed to grayscale
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        clahe_image = clahe.apply(gray)

        landmarks = self.get_landmarks(clahe_image)

        cv2.imshow("image", self.frame) #Display the frame

        if cv2.waitKey(1) & 0xFF == ord('q'): #Exit program when the user presses 'q'
            self.detect = False

    def run(self):
        while not rospy.is_shutdown():
            if self.detect:
                self.face_detect()

if __name__ == '__main__':
    fd = FaceDetect()
    #fd.face_detect()
    fd.run()
