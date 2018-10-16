#!/usr/bin/env python
''' Handwriting Recognition Test Data Building Program
    Author: Matthew Brucker
    Email: matthew.brucker@students.olin.edu
    Maintainer: Connor Novak
    Email: connor@students.olin.edu
    Purpose: Build files of characters for handwriting_recognition.py to use as test data
    '''
''' NOTES:
    - ROI stands for Region Of Interest
    '''

import rospy
import rospkg
import time

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from scipy import stats
from Character import Character

import cv2
import img_processing as Process

class BuildData:

    def __init__(self):
        cv2.setUseOptimized(True)
        self.img = cv2.imread('test_imgs/digits.png')
        rospy.init_node('handwriting_recognition', anonymous=True) # Initialize self as node
        self.bridge = CvBridge()
        rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback) # Subscribe from USB cam
        rospack = rospkg.RosPack()
        self.PACKAGE_PATH = rospack.get_path("edwin")
        self.detect = True
        cv2.namedWindow('image')
        cv2.setMouseCallback('image',self.fill_test_data) # calls fill_test_data() when image clicked
        self.test_data = np.zeros((200,200),np.uint8)
        self.test_filled = 0


    def fill_test_data(self, event, x, y, flags, param):
        '''
            DESC: When mouse button is pressed, builds new data image or saves built data image
            ARGS:
            self - BuildData object on which function is run
            event - cv2 event describing mouse button press
            x - ?
            y - ?
            flags - ?
            param - ?
            RTRN: none
            SHOW: builds and saves test data images in test_imgs file
            '''

        path = self.PACKAGE_PATH + '/params/more_chars/'

        # If mouse button pressed = left button: Build new test data image
        if event == cv2.EVENT_LBUTTONDOWN:
            for contour in self.numbers: # For letter in ROI (see run() function)
                if self.test_filled < 100:
                    x_index = (self.test_filled%10)*20
                    y_index = (self.test_filled // 10)*20
                    self.test_data[y_index:y_index+20,x_index:x_index+20] = contour.img
                    self.test_filled += 1

        # If mouse button pressed = right button: Saves new test data image over current image
        elif event == cv2.EVENT_RBUTTONDOWN:
            ans1 = raw_input('Do you want to save the data?  press y/n \n')
            if(ans1 == 'y') or (ans1 == 'Y'):
                character = raw_input('Save as: __.png')
                write_path = path + character + '.png'
                cv2.imwrite(write_path,self.test_data)
                print('Image saved: ' + write_path)
            elif(ans1 == 'n') or (ans1 == 'N'):
                print('Data not saved')
            else:
                print('Unknown Input:',ans1)

            ans2 = raw_input('Do you want to reset the data? press y/n \n')
            if(ans1 == 'y') or (ans1 == 'Y'):
                self.test_filled = 0 # Sets test_filled counter to zero
                self.test_data[:,:] = 0 # Re-writes test_data to all zeros
                print('Data reset')
            elif(ans1 == 'n') or (ans1 == 'N'):
                print('Data not reset')
            else:
                print('ERROR: Unknown input:',ans1)


    def img_callback(self, data):
        '''
            DESC: Run for every image receipt by usb_cam; converts usbcam image
            to opencv image and saves for further access
            ARGS:
            self - BuildData object on which function is run
            data - current frame from usb cam
            RTRN: none
            SHOW: none
            '''
        try:
            self.curr_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


    def update_frame(self):
        '''
            DESC: saves current frame to self.frame; updates current frame
            ARGS:
            self - BuildData object on which function is run
            RTRN: none
            SHOW: none
            '''
        self.frame = self.curr_frame


    def output_image(self):
        '''
            DESC: Displays current image
            ARGS:
            self - BuildData object on which function is run
            RTRN: none
            SHOW: test data image, final image
            '''
        new_frame = self.frame
        cv2.imshow('image', new_frame)
        cv2.imshow('out',self.test_data)
        cv2.waitKey(1)


    def run(self):
        r = rospy.Rate(10)
        time.sleep(2)
        while not rospy.is_shutdown():
            self.update_frame()
            self.numbers = Process.get_text_roi(self.frame)
            self.output_image()
            r.sleep()


if __name__ == '__main__':
    hr = BuildData()
    hr.run()
