#!/usr/bin/python

#### Object Tracking Code ####
# Lydia Zuehsow, Olin College of Engineering Oct 2016
# This code takes in a single video input and tracks significant objects, identifying them with a green circle. 
# Objects are determined to be significant based on the area of the contour that they produce.
# The background is identified unpon startup, and objects are tracked by their change in position relative to that static image.

import cv2
import imutils
import numpy as np
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from background_subtraction import BackgroundSubtractor

from match import Matcher

MIN_AREA = 10000
MAX_DIST = 10
k_dilate = cv2.getStructuringElement(cv2.MORPH_DILATE, (5,5),(2,2))

matcher = Matcher()

def within(low,x,high):
    return low < x and x < high

def x2t(x, dim, fov):
    theta = fov/2
    # angle difference
    a = float(x) / dim # assuming 640 is frame width
    return np.arctan2(2*a*np.sin(theta)-np.sin(theta), np.cos(theta))

def cnt_avg_col(cnt, img):
    mask = np.zeros(img.shape[:-1], np.uint8)
    cv2.drawContours(mask, [cnt],0,255,-1)
    return cv2.mean(img, mask)

def areacalc(contour, dist):
    xdim, ydim = 640, 480
    xFoV, yFoV = np.deg2rad(65), np.deg2rad(29.2)

    x,y,w,h = cv2.boundingRect(contour)
    print 'x : {}, y : {}, w : {}, h : {}'.format(x,y,w,h)

    centerx = x+(0.5*w)
    centery = y+(0.5*h)

    xmint = x2t(x, xdim, xFoV)
    xmaxt = x2t(x+w, xdim, xFoV)

    ymint = x2t(y, ydim, yFoV)
    ymaxt = x2t(y+h, ydim, yFoV)

    xmin_width = np.tan(xmint) * dist
    ymin_length = np.tan(ymint) * dist
    xmax_width = np.tan(xmaxt) * dist
    ymax_length = np.tan(ymaxt) * dist

    xwidth = xmax_width - xmin_width
    ylength = ymax_length - ymin_length

    area = xwidth * ylength
    return area

def classify_size(area):
    if (area <= .01): # m^2
        return "small"
    elif (area <= .05):
        return "medium"
    else:
        return "large"

class Target(object):
    def __init__(self):
        # construct the argument parser and parse the arguments
        self.redLower = 0
        self.redUpper = 22.5

        self.redLower1 = 160
        self.redUpper1 = 180

        self.greenLower = 30
        self.greenUpper = 80

        self.blueLower = 100
        self.blueUpper = 120

        self.size = 'medium'
        self.color = 'blue'

    def set(self, size, color):
        self.size = size.lower()
        self.color = color.lower()

    def compute_mask(self, img):
        color = self.color.lower() 
        if color == 'red':
            mask0 = cv2.inRange(img, (self.redLower, 0,0), (self.redUpper,255,255))
            mask1 = cv2.inRange(img, (self.redLower1, 0,0), (self.redUpper1,255,255))
            return mask0 + mask1
        elif color == 'blue':
            return cv2.inRange(img, (self.blueLower, 0,0), (self.blueUpper,255,255))
        elif color == 'green':
            return cv2.inRange(img, (self.greenLower, 0,0), (self.greenUpper,255,255))

    def colorcompare(self, color):
        color = color[0]
        objcolor = None
        if within(self.redLower,color,self.redUpper) or within(self.redLower1,color,self.redUpper1):
            objcolor = "red"
        elif within(self.greenLower,color,self.greenUpper):
            objcolor = "green"
        elif within(self.blueLower,color,self.blueUpper):
            objcolor = "blue"
        return (self.color == objcolor)

    def sizecompare(self, size):
        objsize = classify_size(size)
        return (self.size == objsize)

    def filter(self, obj): # size, color
        color_pass = self.colorcompare(obj.color)
        size_pass = self.sizecompare(obj.size)
        print 'col : {}, size : {}'.format(color_pass, size_pass)

        return self.colorcompare(obj.color) and self.sizecompare(obj.size)
                
class Process_Info(object):
    def __init__(self):
        pass

    def process(self, cmd):
        cmdstr = passed_command

        colors = ['blue', 'red', 'green']
        sizes = ['small', 'medium', 'large']

        cmdwrds = cmdstr.split()

        self.objcolor = "Null"
        self.objsize = "Null"
        self.finalpubval = [] #[x,y,dist]

        return self.findcommands(colors, sizes, cmdwrds)

    def findcommands(self, passed_colors, passed_sizes, cmdwrds):
        for word in cmdwrds:
            for color in colors:
                if word.lower() == color.lower():
                    self.color = color
            for size in sizes:
                if word.lower() == size.lower():
                    self.size = size
        print "Looking for a %s %s thing!".format(self.size, self.color)
        return self.size, self.color

class Locate_Object(object):
	def __init__(self):
            self.objcolor = "Null"
            self.objsize = "Null"
            self.finalpubval = [] #[x,y,dist]

        def apply(self, hsv, mask, dist):
            blurred = cv2.GaussianBlur(hsv, (21, 21), 0)
            (cnts, _) = cv2.findContours(cv2.GaussianBlur(mask.copy(), (25,25), 0), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            objects = []
            for c in cnts:
                area = cv2.contourArea(c) # Fake Area
                if area < MIN_AREA:
                    continue

                ## Find Center of Contour Moment
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                d = dist[cY,cX][2] * 10.0 # ??
                area = areacalc(c,d) # Real Area (corrected for physical dimensions)

                ## Protect Limits
                if (cX >= 640):
                        cX = 640 
                elif (cX <= 0):
                        cX = 1

                color = cv2.addWeighted(cnt_avg_col(c, blurred),0.5, blurred[(cY,cX)],0.5,0.0)
                print color[0]

                x,y,w,h = cv2.boundingRect(c)
                objects.append(Object(blurred[y:y+h,x:x+w], color, (cX,cY), area)) # x-y ordering
            return objects

	def findobject(self):
            temp = False
            objnum = 0

            for obj in t.objectlist:
                    objarea = obj[0]

                    if ((area - 25.0) <= objarea) and ((area + 25.0) >= objarea):
                            print 'Found object:' , objnum
                            # print 'List is now: ' , t.objectlist
                            temp = True

                    objnum += 1
                    
            if temp == False:
                    t.objectlist.append([area,(cX, cY),lo.objcolor])
                    print 'Added object to list:' , [area,(int(center[0]), int(center[1])),str(lo.objcolor)]
                    print lo.objcolor
                    print 'List is now: ' , t.objectlist

class Object(object):
    # data class to hold on to relevant parameters
    def __init__(self, img, color, pos, size):
        self.img = img
        self.color = color
        self.pos = pos
        self.size = size # --> real area
    def __eq__(self, other):
        return matcher.match(self.img, other.img)

class ObjectTracker(object):
    def __init__(self):
        self.objects = [] # list of objects it's discovered/tracked so far
        self.current_objects = []
        self.target = Target()
        self.target.set("medium","blue") 

        self.process_info = Process_Info()
        self.locate_object = Locate_Object()

        #rospy.init_node('coord_data', anonymous=True)
        self.sub = rospy.Subscriber('cmd', String, self.cmd_cb) # Check that this works
        self.pub = rospy.Publisher('obj_pos', Point, queue_size=10)

    def cmd_cb(self, cmd):
        #ROS Passing in commands as strings
        size, color = process_info.process(cmd)
        self.set_target(size,color)

    def set_target(self,size,color):
        # target = filter to use for the object
        # contains color-size bounds information
        self.target.set(size,color) 

    def apply(self, mask, img, dist):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        color_mask = self.target.compute_mask(hsv)
        mask = cv2.bitwise_and(color_mask, mask)
        mask = cv2.dilate(mask, k_dilate, iterations=3)
        cv2.imshow('mask', mask)

        self.current_objects = self.locate_object.apply(hsv, mask, dist)

        target_pos = (320,240)
        target_size = 0
        target_img = None

        matched = False # prefer previously seen objects
        for obj in self.current_objects:
            match = False
            for old_obj in self.objects:
                match = matcher.match(old_obj.img, obj.img)
                if not match:
                    self.objects.append(obj)
                else:
                    pass
                    #if self.target.filter(obj):
                    #    matched = True
                    #    target_pos = obj.pos
                    #    target_img = obj.img
                    #break
            if True: #self.target.filter(obj): # passed filter
                target_size = obj.size
                target_pos = obj.pos
                target_img = obj.img

        self.publish(target_pos)
        print target_size
        return target_pos, target_img

    def publish(self, target_pos):
        point = Point(target_pos[0], target_pos[1], 0)#x,y
        self.pub.publish(point)

if __name__ == "__main__":
    rospy.init_node('object_tracker')
    object_tracker = ObjectTracker()
    rospy.spin()
