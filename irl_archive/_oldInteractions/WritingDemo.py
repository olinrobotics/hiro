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

import sys
#sys.path.insert(0,'~/catkin_ws/src/edwin/scripts/sight')
#import handwriting_recognition
from sight import handwriting_recognition
from motion import arm_write

"""
WritingDemo is a demo in which Edwin does text recognition on a piece of paper with
text written on it. Edwin then writes the text he has recognized on piece of paper placed
on the robot table.
"""
class Game:
    def __init__(self):
        self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
        self.behav_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)

        #handwriting recognizer passed in from brain
        self.recognizer = handwriting_recognition.HandwritingRecognition(True)
        self.recognizer.process_data_svm()
        self.recognizer.train_svm()

        #writer object
        self.writer = arm_write.Writer(True)
        self.frame = None

        time.sleep(2)
        print "Starting WritingDemo"

    def img_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def write_word(self, word):
    	# while not rospy.is_shutdown():
    	msg = Edwin_Shape()

    	msg.shape = word
    	msg.x = -500
    	msg.y = 5700
    	msg.z = -810

        time.sleep(2)
        self.writer.write_callback(msg)

    def run(self):
        print "Starting WritingDemo"
        running = True

        self.behav_pub.publish("R_look")

        prev_word = ""
        recognized_word = ""
        word_sureity_lim = 3
        word_count = 0

        while running:
            if self.frame == None:
                return
            word = self.recognizer.get_image_text(self.frame)
            time.sleep(0.1) #so we're not trying to recognize all the time
            # print "W: ", word
            if not word:
                continue
            if (len(word) > 2):
                if word == prev_word:
                    word_count += 1
                else:
                    prev_word = word
                    word_count = 0

                if (word_count > word_sureity_lim) and (word != recognized_word):
                    print "WORD IS: ", word
                    self.write_word(word)

                    running = False
                    recognized_word = word

        print "Finished with WritingDemo :)"

if __name__ == '__main__':
	rospy.init_node('wd_gamemaster', anonymous = True)
	gm = Game()
	gm.run()
