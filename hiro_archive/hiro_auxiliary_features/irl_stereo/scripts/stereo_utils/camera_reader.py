import threading
import numpy as np
import cv2
import time

class CameraReader(threading.Thread):
    def __init__(self,dev):
        self.dev = dev
        self.lock = threading.Lock()
        self.frame = np.empty((480,640,3), dtype=np.uint8)
        self.running = False 
        self.cap = None
        threading.Thread.__init__(self)

    def run(self):
        while self.running:
            self.lock.acquire()
            ret, _ = self.cap.read(self.frame)
            self.lock.release()
            time.sleep(0.01)

    def __enter__(self):
        self.cap = cv2.VideoCapture(self.dev)
        self.running = True 
        self.start()
        return self

    def __exit__(self,exception_type, exception_value, traceback):
        self.stop()
        if self.cap is not None:
            self.cap.release()

    def read(self,frame=None):
        self.lock.acquire()
        if frame is None:
            frame = self.frame.copy()
        else:
            np.copyto(frame,self.frame)
        self.lock.release()
        return frame

    def stop(self):
        self.running = False

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
class ROSCameraReader(object):
    def __init__(self, topic):
        self.topic = topic
        self.sub = None
        self.bridge = CvBridge()
        self.img = np.zeros((480,640,3), dtype=np.uint8)
    def __enter__(self):
        self.sub = rospy.Subscriber(self.topic, Image, self.img_cb)
        return self
    def __exit__(self,a,b,c):
        if self.sub is not None:
            self.sub.unregister()
            self.sub = None
    def img_cb(self,msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    def read(self,frame):
        np.copyto(frame, self.img)
