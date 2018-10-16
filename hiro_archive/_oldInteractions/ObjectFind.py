#!/usr/bin/env python

# Imports
import roslib
import cv2
import rospy
import sys
import math
import time
import numpy as np
import random
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

#####

##### STRUCTURE
###   assume watch position                                                 HoldPos(self)
###   look for objects                                                      
###   determine objects' location in realspace
###   determine optimal route to reach "final position" above object.
###   publish route to node


"""ObjectFind class:
    A class that instantiates and starts looking for specified objects. Contains state
    variables describing the locations of the requested objects and functions to analyze
    video data and relocate Edwin to directly above a requested object.
    | see | | terminal output describing running processes
"""


class ObjectFind:

    """__init__ function
        __init__ is a function run once when a new ItemFind instance is
        created. The function initializes variables to store goal and obj
        positions and other information. It also starts publisher and
        subscriber nodes necessary to run Edwin.
    """
    def __init__(self, init=False):

        print ("INIT| Initializing")
        # ---------- State Variables ----------

        self.debug = True # Debug State
        self.temp = 0
        self.cv_image = None

        # Positions
        self.obj_pos = [0,0]
        self.fin_pos = [0,0]

        # Dimensions
        self.screen_width = 640
        self.screen_height = 480
        self.objfield_top = 6700
        self.objfield_bottom = 3200
        self.objfield_left = -2500
        self.objfield_right = 3000

        # Motion Limits
        self.lowlimit_x = -1500
        self.highlimit_x = 2300
        self.lowlimit_y = 3200
        self.highlimit_y = 6700
        self.lowlimit_z = -1000
        self.highlimit_z = 4000

        # Game.py Elements
        self.timecounter = 0
        self.human_in_frame = False   # False if Edwin does not detect human, True if he does
        self.object_in_frame = False     # False if Edwin does not detect object, True if he does
        self.goal_in_frame = False    # False if Edwin does not detect goal, True if he does
        self.turn_in_progress = False # False if human not taking turn, True otherwise
        self.game_state = 1           # 0 if single-player, 1 if two-player
        self.game_turn = 0            # marks turns goal/reset or edwin/player

        if not init:
            rospy.init_node('push_obj') # Creates node from which to subcribe and publish data

        # Sends data to Edwin
        self.behav_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
        self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=10)

        # Gets data from usb_cam
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback) # Determines node for subscription


    """ HoldPos function:
        Function: moves Edwin to a standardized position where he can examine the entire table
        ----------------------------------------------------------------------------------
        |param | self | access to the state variables of the class calling the function  |s                                         |
    """
    def HoldPos(self):
        time.sleep(1)
        msg1 = "create_route:: R_vulture; 621, 4832, 3739, 845, 195, 0" # Positions multiplied by 10 from #s at top of code
        print ("Sending route: ", msg1)
        self.arm_pub.publish(msg1)
        time.sleep(1)

        # Moves Edwin along route
        self.run_route("R_vulture")
        print ("INIT| Finished")


    def run_route(self, route):
        ##### Publishes the input route to arm_pub.
        msg2 = "run_route:: " + route
        print ("RUN| Sending: ", msg2)
        self.arm_pub.publish(msg2)
        time.sleep(2)


    def callback(self, data):
        ##### Runs once every time an image from usb_cam is received. Converts camera feed into usable data.
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converts USB cam feed ROSmsg to CSV feed. "bgr8" is a type of image encoding.

            # Sets image size to size of received image.
            image_size = self.cv_image.shape
            screen_height = image_size[0]
            screen_width = image_size[1]

        except CvBridgeError as e:
            print(e)


    def apply_filter(self, feed):
        ### Applies a Gaussian blur to the video frame, reducing noise.
        if feed == None:
            return

        blur = cv2.GaussianBlur(feed, (5,5), 0) # Gaussian Blur filter

        # Calls functions to contour obj and calculate moments
        contour, contours = self.contour_feed(blur)

        # Returns contoured feed only if 1+ contours present in image, else runs raw feed
        if len(contours) > 0:
            video = self.calculate(contour, contours)
            cv2.circle(video,(self.obj_pos[0],self.obj_pos[1]),5,(0,0,255),-1) # Draws tracking dots
            # cv2.circle(video,(self.fin_pos[0],self.fin_pos[1]),5,(0,0,0),-1)
        else:
            video = contour

        # Feed Display(s) for debug:
        #cv2.imshow('Raw Feed (feed)',feed)
        #cv2.imshow('Gaussian Blur Filter (blur)', blur)
        #cv2.imshow('Contour Filter (contour)', contour)

        cv2.imshow('Final Contours (video)', video) # Final Contour feed
        k = cv2.waitKey(5) & 0xFF
        return video

    # Contours video feed frame
    def contour_feed(self, video):
        ##### FFFFFFF Replace with our background subtraction code

        contour = video # Duplicate video feed so as to display both raw footage and final contoured footage

        # Changes BGR video to GRAY and dynamically thresholds it [2]
        vidgray = cv2.cvtColor(video,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(vidgray,100,200,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        # Uses kernel to clean noise from image (2x) [1]
        kernel = np.ones((5, 5),np.uint8)
        opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)

        # Cleans out background through extra dilations (3x)
        sure_bg = cv2.dilate(opening,kernel,iterations=3)

        # Calculates contours
        contours, h = cv2.findContours(opening,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        # Creates list of contours with more points than 100 so as to select out for obj and hand
        # finalcontours = [None]*3 # 1st Elem: obj | 2nd Elem: Goal | 3rd Elem: Hand
        finalcontours = [None]
        for cnt in contours:
            area_real = cv2.contourArea(cnt)
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            area_approx = math.pi*radius**2
            area_diff = area_approx - area_real
            diff_coefficient = area_diff / radius
            #cv2.circle(contour,(int(x),int(y)),int(radius),(0,0,255),2) # Draws circles used for area comparison

            finalcontours[0] = cnt #Object

            # # If contour is really close to circle
            # if (diff_coefficient < 100) and (area_real > 200):
            #     if (area_real < 600):
            #         finalcontours[1] = cnt #Goal
            #     elif (area_real > 5000):
            #         finalcontours[0] = cnt #Object
            # if (area_real > 25000):
            #         finalcontours[2] = cnt #Hand


        #if self.debug == True:
            #cv2.drawContours(contour, contours, -1, (255,0,0), 3)
            #cv2.imshow('contour_feed: Final Video(contour)',contour)
        if self.object_in_frame == True: cv2.drawContours(contour, finalcontours[0], -1, (0,0,255), 3)
        # if self.goal_in_frame == True: cv2.drawContours(contour, finalcontours[1], -1, (0,0,0),3)
        # if self.human_in_frame == True: cv2.drawContours(contour, finalcontours[2], -1, (0,255,0),3)

        # Feed Display(s) for debug:
        #cv2.imshow('contour_feed: Raw Video(video)',video)
        #cv2.imshow('contour_feed: To GRAY Filter (vidgray)',vidgray)
        #cv2.imshow('contour_feed: Threshold Filter (thresh)',thresh)
        #cv2.imshow('contour_feed: Opening Kernel (opening)', opening)
        #cv2.imshow('contour_feed: Background Clear (sure_bg)', sure_bg)

        return contour, finalcontours

    # Center & Movement Detection Function
    def calculate(self, contour, finalcontours):

        video = contour

        #--------------------Unpacks finalcontours--------------------#

        # obj contour
        if (finalcontours[0] != None):
            obj_contour = finalcontours[0]
            self.object_in_frame = True
            obj_moments = cv2.moments(obj_contour)

            # Calculates xy values of centroid
            if obj_moments['m00']!=0:
                self.obj_pos[0] = int(obj_moments['m10']/obj_moments['m00'])
                self.obj_pos[1] = int(obj_moments['m01']/obj_moments['m00'])
        else:
            self.object_in_frame = False

        # # Goal contour
        # if (finalcontours[1] != None):
        #     goal_contour = finalcontours[1]
        #     self.goal_in_frame = True
        #     goal_moments = cv2.moments(goal_contour)

        #     # Calculates xy values of centroid
        #     if goal_moments['m00']!=0:
        #         self.fin_pos[0] = int(goal_moments['m10']/goal_moments['m00'])
        #         self.fin_pos[1] = int(goal_moments['m01']/goal_moments['m00'])
        # else:
        #     self.goal_in_frame = False

        # Hand contour
        if (finalcontours[2] != None):
            hand_contour = finalcontours[2]
            self.hand_in_frame = True
        else:
            self.hand_in_frame = False

        # Feed Display(s) for debug:
        #cv2.imshow('calculate: Raw Video(contour)',contour)
        #cv2.imshow('calculate: Centroid Draw(video)',video)

        return video

    """ calibrate function:
        calibrate displays values used in tandem with test_arm_pub.py to calibrate convert_space()
    """
    def calibrate(self):
        self.apply_filter(self.cv_image)
        print("CLB| Obj Pos Screenspace: ", self.obj_pos)
        print("CLB| Object Pos Realspace: ", self.convert_space(self.obj_pos))
        print("CLB| Edwin Final Pos Realspace: ", self.convert_space(self.fin_pos))
        time.sleep(1)

    """ check_pos function:
        check_pos ensures that moving Edwin to a position doesn't breach the limits placed on his movement
    """
    def check_pos(self,pos):
        safe = False
        if (self.lowlimit_x < pos[0] < self.highlimit_x):
            if (self.lowlimit_y < pos[1] < self.highlimit_y):
                if (self.lowlimit_z < pos[2] < self.highlimit_z):
                    safe = True
        return safe

    """ convert_space function
        convert_space converts an xy point in camspace (pixel location) to
        realspace (edwin head location)
        ------------------------------------------------------------------------
        |param  | self | access to the state variables of the class calling    |
        |the function                                                          |
        |param  | x    | x position of point in camspace                       |
        |param  | y    | y position of point in camspace                       |
        |return |      | vector of obj x and y position in realspace           |
    """
    def convert_space(self, pos):
        ###### FFFFFFF Write calibration function.

        if self.debug == True: print("SPC: Old Coordinates: ", pos[0], pos[1])

        # Determines equations to convert from camspace to realspace in x-direction
        m1 = (2000 - 300)/(484 - 305)
        b1 = - 2000
        x_real = m1 * pos[0] + b1

        # Determines equations to convert from camspace to realspace in Y-direction
        m2 = (6900 - 4900)/(127 - 352)
        b2 = 7500
        y_real = m2 * pos[1] + b2

        if self.debug == True: print("SPC: New Coordinates: ", x_real, y_real)
        return([x_real, y_real])

    # """ push_obj function:
    #     push_obj makes Edwin push the obj to the goal
    #     ---------------------------------------------------------------------------------
    #     |param | self | access to the state variables of the class calling the function |
    #     |see   |      | Edwin pushes or pulls the obj such that it covers the goal      |
    # """
    # def push_obj(self):

    #     # Converts game piece positions into realspace
    #     obj = self.convert_space(self.obj_pos)
    #     # goal = self.convert_space(self.fin_pos)
    #     if self.debug == True: # Debugging info
    #         print ("PSH| pos = ", obj)
    #         print ("PSH| goal pos = ", goal)

    #     # Checks if positions are safe [TOFIX]
    #     if (self.check_pos([obj[0], obj[1], -500]) == True):

    #     #---------------ROUTE CONSTRUCTION---------------#
    #         # Determines direction to push obj
    #         direction = None
    #         msg1 = "create_route:: R_push; "
    #         if obj[0] < goal[0]: direction = "Right"
    #         if obj[0] > goal[0]: direction = "Left"

    #         if obj[1] < goal[1]: # Position Below obj; Push Up to Goal Y
    #             print("PSH| Pushing Up")
    #             msg1 = msg1 + str(obj[0]) + ", " + str(obj[1] - 300) + ", -800, 845, 195, 0"                    # Head Down
    #             msg1 = msg1 + ", " + str(obj[0]) + ", " + str(goal[1]) + ", -800, 845, 195, 0"                  # Push Forward
    #             msg1 = msg1 + ", " + str(obj[0]) + ", " + str(goal[1] - 100) + ", -800, 845, 195, 0"            # Disengage

    #             if direction == "Left": # Position Right of obj; Push Left to Goal X
    #                 print("PSH| Pushing Left")
    #                 msg1 = msg1 + ", " + str(obj[0] + 800) + ", " + str(goal[1] + 800) + ", -800, 845, 195, 0"        # Move Right
    #                 msg1 = msg1 + ", " + str(obj[0] + 800) + ", " + str(goal[1]) + ", -800, 845, 195, 0"  # Move Up
    #                 msg1 = msg1 + ", " + str(goal[0] + 300) + ", " + str(goal[1]) + ", -800, 845, 195, 0"       # Push Left
    #             elif direction == "Right": # Position Left of obj; Push Right to Goal Y
    #                 print("PSH| Pushing Right")
    #                 msg1 = msg1 + ", " + str(obj[0] - 300) + ", " + str(goal[1]) + ", -800, 845, 195, 0"        # Move Left
    #                 msg1 = msg1 + ", " + str(obj[0] - 300) + ", " + str(goal[1] + 300) + ", -800, 845, 195, 0"  # Move Up
    #                 msg1 = msg1 + ", " + str(goal[0]) + ", " + str(goal[1] + 300) + ", -800, 845, 195, 0"       # Push Right
    #         else: # Position Above obj; Push Down to Goal Y
    #             print("PSH| Pushing Down")
    #             msg1 = msg1 + str(obj[0]) + ", " + str(obj[1] + 800) + ", -1000, -24, 240, 0"                   # Head Down
    #             msg1 = msg1 + ", " + str(obj[0]) + ", " + str(obj[1] + 800) + ", -1000, 845, 195, 0"            # Chin Down
    #             msg1 = msg1 + ", " + str(obj[0]) + ", " + str(goal[1] + 1000) + ", -800, 845, 195, 0"           # Push Backward
    #             msg1 = msg1 + ", " + str(obj[0]) + ", " + str(goal[1] + 1100) + ", -800, 845, 195, 0"           # Disengage

    #             if direction == "Left": # Position Right of obj; Push Left to Goal X
    #                 print("PSH| Pushing Left")
    #                 msg1 = msg1 + ", " + str(obj[0] + 800) + ", " + str(goal[1] + 1100) + ", -800, 845, 195, 0" # Move Right
    #                 msg1 = msg1 + ", " + str(obj[0] + 800) + ", " + str(goal[1]) + ", -800, 845, 195, 0"        # Move Down
    #                 msg1 = msg1 + ", " + str(goal[0]) + ", " + str(goal[1]) + ", -800, 845, 195, 0"             # Push Left
    #             elif direction == "Right": # Position Left of obj; Push Right to Goal Y
    #                 print("RSH| Pushing Right")
    #                 msg1 = msg1 + ", " + str(obj[0] - 800) + ", " + str(goal[1] + 1100) + ", -800, 845, 195, 0"       # Move Left
    #                 msg1 = msg1 + ", " + str(obj[0] - 800) + ", " + str(goal[1]) + ", -800, 845, 195, 0" # Move Down
    #                 msg1 = msg1 + ", " + str(goal[0]) + ", " + str(goal[1]) + ", -800, 845, 195, 0"      # Push Right

    #         # Creates and runs route, then returns to overview
    #         print ("PSH| Sending: ", msg1)
    #         self.arm_pub.publish(msg1)
    #         time.sleep(1)
    #         self.run_route("R_push")
    #         self.run_route("R_vulture")

    # """ reset_obj function:
    #     reset_obj makes Edwin push the obj to a "random" point on the gameboard
    #     ---------------------------------------------------------------------------------
    #     |param | self | access to the state variables of the class calling the function |
    #     |see   |      | Edwin pushes or pulls the obj such that it is no longer covering|
    #     |the goal.                                                                      |
    # """
    # def reset_obj(self):
    #     pos = self.convert_space(self.obj_pos)
    #     print("RST| pos = ", pos)

    #     new_pos = [random.randrange(self.gameboard_left,self.gameboard_right), random.randrange(self.gameboard_bottom, self.gameboard_top)]
    #     print("RST| new pos = ", new_pos)
    #     msg = "create_route:: R_reset; "
    #     direction = None
    #     if new_pos[0] < pos[0]: direction = "Left"
    #     if new_pos[0] > pos[0]: direction = "Right"

    """ play_game function
        play_game holds Edwin's game logic and makes him decide when to move
        and act. The function calls Edwin's physical moving functions at the
        appropriate times.
        ---------------------------------------------------------------------------------
        |param | self | access to the state variables of the class calling the function |
    """
    def play_game(self):

        if self.debug == True: print("DBG| Locating Object Loop")
        self.apply_filter(self.cv_image)
        time.sleep(2)

        if self.game_state == 0: # If Edwin is playing with another person
            if self.game_turn == 1: # If it's Edwin's turn
                if self.object_in_frame == True: # If game pieces are on the playing field
                    if self.debug == True: print ("Pushing obj . . .")
                    self.push_obj()
                    self.game_turn = 0
                    if self.debug == True: print("Turn: Player")

                else: # If some game pieces are missing
                    if self.debug == True:
                        if self.object_in_frame == False: print("ERROR: No object in frame")
                        if self.goal_in_frame == False: print("ERROR: No goal in frame")
            else: # If it's the player's turn
                if self.human_in_frame == True and self.turn_in_progress == False: # Human starts taking turn
                    self.turn_in_progress = True
                if self.human_in_frame == False and self.turn_in_progress == True: # Human has ended turn
                    self.turn_in_progress = False
                    self.game_turn == 1
                    if self.debug == True: print("Turn: Edwin")
                    time.sleep(1)


        else: # If Edwin is playing by himself
            if self.game_turn == 0: # If it's Edwin's goal turn
                print("PLY| Goal Turn")
                if self.object_in_frame == True: # If game pieces are on the playing field
                    print ("PLY| Pushing obj")
                    self.push_obj()
                    time.sleep(10)
                    # self.check_win(self.goal_in_frame)
                    self.game_turn = 1

                else: # If some game pieces are missing

                    if self.debug == True:
                        if self.object_in_frame == False: print("DBG| ERROR: No obj in frame")
                        ### FFFFFFFFF Need to check if action is possible given arm constraints.
                        # if self.goal_in_frame == False: print("DBG| ERROR: No goal in frame")
            else: # If it's Edwin's reset turn
                if self.object_in_frame == True: # If game pieces are on the playing field
                    print("PLY| Reset Turn")
                    self.reset_obj()
                    time.sleep(5)
                    self.game_turn = 0
                else: # If some game pieces aren't in position
                    if self.debug == True:
                        if self.object_in_frame == False: print("DBG| ERROR: No obj in frame")
                        if self.goal_in_frame == True: print("DBG| ERROR: Goal not covered by obj")

    """ run function:
        run handles the actual updating of the code and continuation of the program
        ---------------------------------------------------------------------------------
        |param | self | access to the state variables of the class calling the function |
    """
    def run(self):
        r = rospy.Rate(20) # Sets update rate
        while not rospy.is_shutdown():
            r.sleep()
            # if self.debug == True: self.calibrate()
            self.play_game()


if __name__=='__main__':
    print "in main"
    of = ObjectFind() # Creates new instance of class PushobjGame
    of.run() # Calls run function
