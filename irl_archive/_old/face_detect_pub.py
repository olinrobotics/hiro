#!/usr/bin/python
import cv2
import time
import Image
import roslib; roslib.load_manifest('edwin')
import rospy
from std_msgs.msg import String
import numpy as np

class FaceFinder:
    def __init__(self):
        rospy.init_node("face_finder", anonymous=True)
        self.pub = rospy.Publisher('behavior_cmd', String, queue_size=10)

        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_alt.xml') #face xml
        self.mouth_cascade = cv2.CascadeClassifier('haarcascade_smile.xml') #smile xml

    def find_smile(self):
        ret, self.frame = self.cap.read()
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        real_faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.2, minSize=(20,20))

        if len(real_faces) != 0:
            for (x,y,w,h) in real_faces:
                print "FACE"
                self.pub.publish("greet")
                cv2.rectangle(self.frame,(x,y),(x+w,y+h),(0,0,255),2)
                roi_gray = gray[y:y+h, x:x+w]
                roi_color = self.frame[y:y+h, x:x+w]
                mouth = self.mouth_cascade.detectMultiScale(roi_gray, scaleFactor=1.7, minNeighbors=20, minSize=(10,10), flags=cv2.cv.CV_HAAR_SCALE_IMAGE)
                for (mp,mq,mr,ms) in mouth:
                    cv2.rectangle(roi_color,(mp,mq),(mp+mr,mq+ms), (255,0,0),1)
                    print "SMILE"

        cv2.imshow("output", self.frame)
        c = cv2.waitKey(1)



    def run(self):
        self.cap = cv2.VideoCapture(1)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.find_smile()
            r.sleep()


if __name__ == "__main__":
    face_finder = FaceFinder()
    face_finder.run()
