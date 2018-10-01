#!/usr/bin/env python

# import the necessary packages
import imutils
import rospy
#from PIL import Image
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from irl.msg import blue_button, yellow_button, red_button, green_button, all_buttons
import numpy as np
import cv2

from cv_bridge import CvBridge, CvBridgeError

"""
- Written by: Mark Goldwater
- What it does: Tracks location and radius of red, green, blue, and yellow buttons
and publishes them to 'button data' topic
- Dependencies: You need the custom messages I created (after from irl.msg) in you CMakeLists.txt file
- How to Run: Run individually using 'python button_tracker.py'
"""

class buttonTracker(object):

    """
    Trackes the buttons and publishes their information to a topic
    """

    def __init__(self):
        self.node = rospy.init_node("button_tracker")
        #Eself.buttonPublisher = rospy.Publisher('button_data', all_buttons, queue_size=10)

        #self.blue_button = blue_button()
        #self.red_button = red_button()
        #self.green_button = green_button()
        #self.yellow_button = yellow_button()

        #self.all_buttons = all_buttons()

        self.greenLower = (29, 86, 6)
        self.greenUpper = (64, 255, 255)

        self.blueLower = (110, 50, 50)
        self.blueUpper = (130, 255, 255)

        self.yellowLower = (23, 41, 133)
        self.yellowUpper = (40, 150, 255)

        self.redLower = (160, 140, 50)
        self.redUpper = (179, 255, 255)

        self.bridge = CvBridge()
        self.obtained_frame = False
        self.frame = None
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.make_frame,queue_size = 10)

        # grab the reference to the webcam
        #self.camera = cv2.VideoCapture(0)

    def make_frame(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.obtained_frame = True
        except CvBridgeError, e:
            self.ontained_frame = False

            print e

    def run(self):
        #keep looping
        while(True):
            if (self.obtained_frame):
                allButtons = []

                #grab the current frame
                #(grabbed, frame) = self.frame
                frame = self.frame
                cv2.imshow("Frame_raw", frame)

                # resize the frame, blur it, and return
                # it in the HSV color space
                frame = imutils.resize(frame, width=600)
                blurred = cv2.GaussianBlur(frame, (11, 11), 0)
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                # create a mask for the color green and then
                # perform erodions and dialations to make the
                # tracking more smooth

                mask_blue = cv2.inRange(hsv, self.blueLower, self.blueUpper)
                mask_green = cv2.inRange(hsv, self.greenLower, self.greenUpper)
                mask_yellow = cv2.inRange(hsv, self.yellowLower, self.yellowUpper)
                mask_red = cv2.inRange(hsv, self.redLower, self.redUpper)

                masks = [mask_blue, mask_green, mask_yellow, mask_red]

                    # mask_total = mask_blue + mask_green + mask_yellow + mask_red"""

                for i in range(4):
                    masks[i] = cv2.erode(masks[i], None, iterations=2)
                    masks[i] = cv2.dilate(masks[i], None, iterations=2)

                    # find contours in the mask and initialize the (x, y) position
                    # of the center of the ball
                    cnts = cv2.findContours(masks[i].copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
                    center = None

                    # only proceed if at least one contour found
                    if len(cnts) > 0:
                        # find largest contour area, compute minimum enclosing circle
                        # and find its center
                        c = max(cnts, key=cv2.contourArea)
                        ((x, y), radius) = cv2.minEnclosingCircle(c)
                        M = cv2.moments(c)
                        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                        # only proceed if radius meets minimum size
                        if radius > 10:
                            if i == 0:
                                blue_center = center
                                blue_radius = radius

                                allButtons.append(blue_center)

                                #self.blue_button.center_x = center[0]
                                #self.blue_button.center_y = center[1]
                                #self.blue_button.radius = radius
                            elif i == 1:
                                green_center = center
                                green_radius = radius

                                allButtons.append(green_center)

                                #self.green_button.center_x = center[0]
                                #self.green_button.center_y = center[1]
                                #self.green_button.radius = radius
                            elif i == 2:
                                yellow_center = center
                                yellow_radius = radius

                                allButtons.append(yellow_center)

                                #self.yellow_button.center_x = center[0]
                                #self.yellow_button.center_y = center[1]
                                #self.yellow_button.radius = radius
                            elif i == 3:
                                red_center = center
                                red_radius = radius

                                allButtons.append(red_button)

                                #self.red_button.center_x = center[0]
                                #self.red_button.center_y = center[1]
                                #self.red_button.radius = radius


                        #self.all_buttons.blue_button = blue_button
                        #self.all_buttons.green_button = green_button
                        #self.all_buttons.yellow_button = yellow_button
                        #self.all_buttons.red_button = red_button

                        #self.buttonPublisher.publish(all_buttons)


                        # draw circle outline and cente
                        cv2.circle(frame, (int(x), int(y)), int(radius), (0,255,255), 2)
                        cv2.circle(frame, center, 5, (0, 0, 255), -1)



                # show frame to screen
                cv2.imshow("Frame", frame)
                key = cv2.waitKey(1) & 0xFF

                print(allButtons)
            #else:
                #print("No Frame!")
        #return allButtons

        # q key will stop loop
        #    if key == ord("q"):
        #        break

    #turn off camera and close any open windows
    #camera.release()
    #cv2.destoryAllWindows



if __name__=="__main__":
    Tracker = buttonTracker()
    Tracker.run()
