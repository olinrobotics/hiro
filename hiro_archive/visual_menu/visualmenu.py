#!/usr/bin/env python
import rospy
import cv2
import cv2.cv as cv
import numpy as np
from scipy import stats
import math

from std_msgs.msg import String
from sensor_msgs.msg import Image
from edwin.msg import Edwin_Shape
from cv_bridge import CvBridge, CvBridgeError

#Todo: Increase tracking accuracy


class visualMenu:
    #The goal is to have someone put their hand under the camera, and then
    #the camera will read which portion of the screen the hand is in.
    #If the person keeps their hand relatively still for 3 seconds,
    #determined by increasing the contour area by 50%, then the
    #Dimensions of a camera screen are 640 x 480.

    def __init__(self):
        #rospy.init_node('visualmenu')
        #self.choice_pub = rospy.Publisher("vm_choice", String, queue_size=10)
        #self.game_state = rospy.Subscriber("all_control", String, self.cmd_callback)


        self.bridge = CvBridge()
        rospy.Subscriber("usb_cam/camera_raw", Image, self.img_callback)

        self.choice = ""
        self.bg_sub = None
        self.detect = True
        self.frame = None
        self.mode = 1 #mode1 = local camera, mode 2 = ROS camera feed.
        self.cap = cv2.VideoCapture(0)

        self.median = []
        self.region_list = []
        self.area_list = []
        self.colorList = self.calibrate_color()
        self.decision_length = 40 #amount of time in the recorded queue to m
                                    #Decision.




        self.button_point_list = [(100, 240), (320,240), (540, 240)]
        self.activities_list = ["SimonSays: Player", "SimonSays: Simon", "Homework"]

        #Initialize buttons:
        for i in self.button_point_list:
            self.create_button(i)

############
#ROS section
############

    def img_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #h, w = self.frame.shape[:2]
        except CvBridgeError as e:
            print(e)

    def cmd_callback(self, data):
        print "GOT: ", data.data
        if "vm stop" in data.data:
            self.detect = False
        elif "vm go" in data.data:
            self.started_tracking = time.time()
            self.detect = True

#################
#Image Processing
#################

    def findHand(self, frame):

        composite = frame[0]
        blur = frame[1]
        #Find the hand
        max_area = 0

        #composite = self.bg_sub.apply(blur.copy())

        contours, hierarchy = cv2.findContours(composite.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for i in range(len(contours)):
            cont=contours[i]
            area = cv2.contourArea(cont) #Theoretically the largest contour is a hand.
            if(area>max_area):
                max_area=area
                ci=i

        cont = contours[ci]

        #find center coordinate of the hand, or rather, the centroids
        m = cv2.moments(cont)
        cx = int(m['m10']/m['m00'])
        cy = int(m['m01']/m['m00'])

        coords = (cx,cy)

        #cv2.circle(blur, coords, 10, (255, 0, 0), 2)
        cv2.imshow("composite", composite)
        #cv2.imshow("blur", blur)
        cv2.waitKey(1)
        return coords

    def process_frame(self, frame, cList):
        #Image pre-processing
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blur = cv2.GaussianBlur(frame,(5,5),0)
        #ret,thresh1 = cv2.threshold(blur,10,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        #Kalman filter?

        frame_list = [];

        for i in cList:
            blur_copy = cv2.cvtColor(blur.copy(), cv2.COLOR_BGR2HSV)
            colorHigh = i.copy()
            colorHigh[0] = colorHigh[0] + 3
            colorHigh[1] = colorHigh[1] + 20
            colorHigh[2] = colorHigh[2] + 40
            colorLow = i.copy()
            colorLow[0] = colorLow[0] -3
            colorLow[1] = colorLow[1] -20
            colorLow[2] = colorLow[2] -40
            #colorHigh = i + 20
            #colorLow = i - 30

            #http://stackoverflow.com/questions/8593091/robust-hand-detection-via-computer-vision

            mask = cv2.inRange(blur_copy, colorLow, colorHigh)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)



            frame_list.append(mask)


        composite = sum(frame_list)
        composite = cv2.medianBlur(composite, 5)

        frames = [composite, blur]

        #cv2.imshow("composite", composite)
        #cv2.waitKey(1)
        return frames

###############
#Initialization
###############


    def calibrate_color(self):
        #With an initial guess, the person calibrates their hand color onto the machine,
        #the loop will auto-adjust until it reaches an "optimal" value, and decides the
        #range from there.
        cList = self.initialize_reference()
        roi_params = (160, 40, 480, 440)#x,y, length, height
        total_pixels = 640*480 - 480*440

        # while True:
        #     k = cv2.waitKey(100) & 0xFF
        #     if k == ord('q'):
        #         cv2.destroyAllWindows()
        #         print cList
        #         return cList
        #     else:
        #         #The auto-adjust functions by reading how much "white" is detected
        #         # in the non-ROI.  It will try to minimize the outside while maximizing
        #         #the inside.
        #         frame = self.get_frame()
        #
        #         composite = self.process_frame(frame, cList)
        #         composite = composite[0]
        #
        #         roi = composite.copy()
        #         roi = roi[roi_params[0]:roi_params[0]+roi_params[2],
        #             roi_params[1]:roi_params[1]+roi_params[3]]
        #
        #         total_black = 640*480 - cv2.countNonZero(composite)
        #         roi_black = 480*440 - cv2.countNonZero(roi)
        #
        #         bg_noise = total_pixels - (total_black - roi_black)
        #         cv2.imshow("composite", composite)
        #         print bg_noise
        print cList
        return cList

                #http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
                #http://simena86.github.io/blog/2013/08/12/hand-tracking-and-recognition-with-opencv/
                #http://www.pyimagesearch.com/2016/02/15/determining-object-color-with-opencv/

    def initialize_reference(self):
        colorGuess = np.array([115,160, 229]) #skin tone guess
        #229,160, 115

        p1 = (325,135)
        p2 = (325,305)
        p3 = (245, 305)
        p4 = (405, 305)
        p5 = (245, 250)
        p6 = (405, 250)

        pList = [p1, p2, p3, p4, p5, p6]


        cList = [[],[],[],[],[],[]]

        # #Background Subtractor initialization
        #
        # while self.detect:
        #     k = cv2.waitKey(1) & 0xFF
        #     if k == ord('q'):
        #         break
        #     else:
        #         frame = self.get_frame()
        #         cv2.putText(frame,
        #         "Vacate the frame and press 'q' when you are ready to calibrate",
        #         (40, 460), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 2)
        #         cv2.imshow("test", frame)
        #
        # self.bg_sub = cv2.BackgroundSubtractorMOG2()


        while self.detect:
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            else:

                frame = self.get_frame()
                #Turn into HSV
                read_frame = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2HSV)


                cv2.putText(frame,
                "Put your hand in the box and press 'q' when you are ready to calibrate",
                (40, 460), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 2)

                #Overall frame for the hand
                cv2.rectangle(frame, (160, 40), (480, 440), colorGuess, 3)

                #Visual marker for each detection point.
                for i in pList:
                    point1 = tuple((np.asarray(i) - 5))
                    point2 = tuple((np.asarray(i) + 5))
                    cv2.rectangle(frame, point1, point2, colorGuess, -1)

                # for i in range(0, len(pList)):
                #     point = pList[i]
                #     #retrieve each RGB value separately, compile into tuple
                #     cList[i] = (int(frame[point][0]), int(frame[point][1]), int(frame[point][2]))



                #process each point
                # cList = np.array(cList)
                # colorGuess = np.mean(cList, axis=0)

                cv2.imshow("test", frame)


            #Once target is set, find the median
        for j in range(0,50):
            frame = self.get_frame()
            #Overall frame for the hand
            cv2.rectangle(frame, (160, 40), (480, 440), colorGuess, 3)

            #Visual marker for each detection point.
            for i in pList:
                point1 = tuple((np.asarray(i) - 5))
                point2 = tuple((np.asarray(i) + 5))
                cv2.rectangle(frame, point1, point2, colorGuess, -1)

            #Read all the points' colors, add to list.
            for k in range(0,len(cList)):
                point = pList[k]
                cList[k].append((int(read_frame[point][0]), int(read_frame[point][1]), int(read_frame[point][2])))


            cv2.imshow("test", frame)
            cv2.waitKey(1)



        for l in range(0,len(cList)): #pare down to finding the mode?
            # med = cList[l]
            # med = np.median(med)
            # med = int(med)
            # cList[l] = med
            cList[l] = np.mean(cList[l], axis=0)

        cv2.destroyAllWindows()
        return cList

################
#General Utility
################

    def check_pos(self, coords):
        xy = coords
        #xy = (640, 240)#testing coordinate.  Seems to falter on the ends of lists.

        for i in self.area_list:
            if xy in i:

                return self.area_list.index(i) #Is there a simpler way?

    def get_frame(self):
        #Mode for switching between
        if self.mode == 1:
            grab, frame = self.cap.read()
            frame = cv2.flip(frame, 1)

            return frame

        elif self.mode == 2 and self.frame != None:
            frame = cv2.flip(self.frame, 1)

            return frame

    def get_median(self, option): #Technically the mode, but that would be confusing.
        if option == None:
            self.median = []

        elif len(self.median) < self.decision_length:
            self.median.append(option)
        elif len(self.median) == self.decision_length:
            self.median.pop(0)
            self.median.append(option)
            med = self.median[:]
            med = stats.mode(med)
            med = int(med[0])

            return med



    def get_area_coords(self, params):
        coord_list = []
        for x in range(params[0][0], params[0][1]+1): #plus one is important.
            for y in range(params[1][0], params[1][1]+1):
                coord_list.append((x,y))

        return coord_list

##################
#Visual components
##################

    def dispRegion(self, frame, coords,option):


        # if option == 0:
        #     cv2.rectangle(frame, (0,0), (210, 480), (255,0,0), -1)
        # elif option == 1:
        #     cv2.rectangle(frame, (211,0), (429, 480), (0,255,0), -1)
        # elif option == 2:
        #     cv2.rectangle(frame, (430,0), (640, 480), (0,0,255), -1)

        x = self.region_list[option][0]
        y = self.region_list[option][1]
        point1 = (x[0], y[0])
        point2 = (x[1], y[1])
        cv2.rectangle(frame, point1, point2, (0,255,0), -1)


        # cv2.putText(frame,
        # str(option),
        # point1, cv2.FONT_HERSHEY_PLAIN, 5, (255,255,255), 2)

        cv2.circle(frame, coords, 10, (255, 0, 0), 2)
        cv2.imshow("Menu", frame)
        cv2.waitKey(1)

    def dispMenu(self, frame, option, coords):
        med = self.get_median(option)
        if med == None: #Nothing selected.
            for j in range(0, len(self.region_list)):
                i = self.region_list[j]
                point1 = (i[0][0], i[1][0])
                point2 = (i[0][1], i[1][1])
                cv2.rectangle(frame, point1, point2, (0,0,255), -1)
                text_spot = list(self.button_point_list[j])
                text_spot[0] = text_spot[0] - 70
                text_spot = tuple(text_spot)
                cv2.putText(frame, self.activities_list[j],
                text_spot, cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 2)

        else:
            i = self.region_list[med]
            point1 = (i[0][0], i[1][0])
            point2 = (i[0][1], i[1][1])
            cv2.rectangle(frame, point1, point2, (0,255,0), -1)

            text_spot = list(self.button_point_list[med])
            text_spot[0] = text_spot[0] - 70
            text_spot = tuple(text_spot)

            cv2.putText(frame, self.activities_list[med],
            text_spot, cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 2)


            cv2.waitKey(200)
            self.run_choice(med)

            #Do publish a certain message of some kind.

        cv2.circle(frame, coords, 10, (255, 0, 0), 2)
        cv2.imshow("Menu", frame)
        cv2.waitKey(1)




    def splitScreen(self, n):
        regions = []

        if n == 3:
            regions = [[[0,210], [0,480]], [[211,429], [0,480]], [[430,640], [0,480]]]

        elif n%7 != 0:
            col = int(math.ceil(math.sqrt(n)))
            row = int(math.ceil(n/float(col)))
            dx = int(640/col)
            dy = int(480/row)

            for x in range(0, col):
                for y in range(0, row):
                    region = [[x*dx,(x+1)*dx],[y*dy,(y+1)*dy]] #same format, minx, maxx, miny, maxy
                    regions.append(region)

        else:
            print "pick a valid number and pray"

        self.region_list = regions
        areas = regions[:] #slice to create a new list

        for i in range(0, len(areas)):
            areas[i] = self.get_area_coords(areas[i])
        self.area_list = areas


    def create_button(self, coords): #Coords is an x, y
        length = 150
        x = coords[0]
        y = coords[1]

        region = [[x-length/2, x+length/2],[y-length/2, y+length/2]]

        self.region_list.append(region)

        self.area_list.append(self.get_area_coords(region))

################s
#Game.py Logic
################

    def run_choice(self, option):
        if option == 0:
            #self.choice_pub.publish("SimonSays:Player")
            self.choice = "SimonSays:Player"
        elif option == 1:
            #self.choice_pub.publish("SimonSays: Simon")
            self.choice = "SimonSays:Simon"
        elif option == 2:
            #self.choice_pub.publish("Homework")
            self.choice = "Homework"

        self.detect = False

################
#Program running
################

    def run(self):
        #r = rospy.Rate(10)
        #time.sleep(2)

        #while not rospy.is_shutdown():

        #Initial Calibration



        while self.detect:
            frame = self.get_frame()
            processed_frame = self.process_frame(frame, self.colorList)
            coords = self.findHand(processed_frame)
            option = self.check_pos(coords)

            self.dispMenu(frame, option, coords)



            # if option == 0:
            #     pass #run this node.
            # elif option == 1:
            #     pass #run the other node.
            # #r.sleep()

        print self.choice
        return self.choice

if __name__ == '__main__':
    vm = visualMenu()
    vm.run()
