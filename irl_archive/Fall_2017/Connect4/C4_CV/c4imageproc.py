#!/usr/bin/env python

'''
Purpose: to process a board of Connect4
Contact: hannah.kolano@students.olin.edu
Last edited: 12/7/17
'''

import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DetectConnectFour:
    '''
    Continuously checks the board
    '''

    def __init__(self, numcols, numrows):
        '''initialize the object'''
        self.ready = 0
        self.numcols = numcols
        self.numrows = numrows
        self.width, self.height = numcols*100, numrows*100

        rospy.init_node('boardDetector')
        self.pub_move = rospy.Publisher('opponent_move', Int16, queue_size=10)
        self.sub_ready = rospy.Subscriber('c4_ready', Int16, self.ready_status, queue_size=10)
        self.sub_camera = rospy.Subscriber('usb_cam/image_raw', Image, self.img_callback)

        self.bridge = CvBridge() # converts image types to use with OpenCV


        print "C4 CV Initialized"

    def ready_status(self, data):
        self.ready = data.data

    def img_callback(self, data):
        ''' DOCSTRING:
        Given img data from usb cam, saves img for later use; called every
        time img recieved from usb cam
        '''
        try:
            # Saves image; converts to opencv format
            self.curr_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print 'ERR:%d',e

    def run(self):
        '''initialize some states'''
        state = 'empty'
        playerColor = 'Nothing'

        while True:
            '''check if Draco is ready to look'''
            # self.ready = raw_input('Ready?')
            if self.ready == 1:
                print "Searching..."
                frame = self.curr_frame
                #picture = raw_input('Choose a picture')
                #frame = cv2.imread(picture)
                #frame = cv2.resize(frame,None,fx=1, fy=1, interpolation = cv2.INTER_AREA)

                '''parse for black'''
                viewsilhouette = self.extract_black(frame)

                '''find the major contour, reduce the field of view'''
                contours = self.draw_contours(viewsilhouette, frame)#, showme = 1)
                x, y, w, h = self.draw_basic_boxes(contours, frame)#, showme = 1)
                board = self.transform_to_grid(frame, np.float32([[x,y],[x,y+h],[x+w,y+h],[x+w,y]]))

                '''warp transform to actual grid'''
                if state == 'empty':
                    actual_corners = self.detect_corners(board, showme = 1)
                board = self.transform_to_grid(board, actual_corners)
                #self.show_image(board, 'after grid')
                current_layout = self.determine_layout(board)
                print(current_layout)

                '''if the board was empty, initialize some variables'''
                if state == 'empty':
                    for column in current_layout:
                        if 'Orange' not in column and 'Blue' not in column:
                            pass
                        else:
                            if 'Orange' in column:
                                playerColor = 'Orange'
                            else:
                                playerColor = 'Blue'
                            print('player color is', playerColor)
                            state = 'started'
                            first_ball = True

                if state == 'started':
                    try:
                        if current_layout != self.layout or first_ball:
                            changed_column, new_color = self.which_column_changed(current_layout, playerColor)
                            changed_column = Int16(changed_column)
                            print('new color is', new_color)
                            if new_color == playerColor:
                                print('changed column', changed_column)
                                self.pub_move.publish(changed_column)
                            first_ball = False
                    except AttributeError as err:
                        print 'I need an empty layout first!'
                        print 'Here is what I got instead:', current_layout

                self.ready = 0
                self.layout = current_layout
                # time.sleep(1)


    def which_column_changed(self, current_layout, playerColor):
        '''DOCSTRING:
        input the current board layout and what color the player is
        checks if each column is not the same as before; outputs the column number and color
        '''
        for i in range(len(current_layout)):
            if current_layout[i] != self.layout[i]:
                for j in range(len(current_layout[i])):
                    if current_layout[i][j] != 'Nothing':
                        return i, current_layout[i][j]

    def draw_basic_boxes(self, contours, frame, showme = 0):
        '''DOCSTRING:
        Input the contours and image they're from
        returns the coordinates of bounding box of the largest one
        (that is not the frame itself)
        '''
        imagewidth, imageheight, __ = frame.shape
        areas = []
        i = 0

        #iterate through the contours
        for cnt in contours:
            x,y,w,h = cv2.boundingRect(cnt)
            if(w*h) >= 10000 and w*h < imagewidth*imageheight-1500:
                areas.append((i, w*h))
            i += 1

        #sort by area
        areas = sorted(areas, key=lambda x: x[1])
        biggest_box = areas[-1]
        x,y,w,h = cv2.boundingRect(contours[biggest_box[0]])

        #ONLY SHOW IF NECESSARY! Corner detection will pick it up unfortunately
        if showme !=0:
            cv2.rectangle(frame,(x-20,y-20),(x+w+20,y+h+20),(0,0,255),2)
            self.show_image(frame, 'biggest_box')
        return x-20, y-20, w+40, h+30

    def draw_contours(self, thresholdedpic, origpic, showme = 0):
        '''DOCSTRING:
        takes a thresholded pictures and the actual thing and finds the contours
        will not draw the contours unless showme=0
        '''
        #TODO:
        ''' check out https://docs.opencv.org/3.3.0/d9/d61/tutorial_py_morphological_ops.html'''
        __, contours, __ = cv2.findContours(thresholdedpic,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(origpic, contours, -1, (0,255,0), 2)
        if showme != 0:
            cv2.drawContours(origpic, contours, -1, (0,255,0), 2)
            self.show_image(origpic, 'contours')
        return contours

    def extract_black(self, frame, showme=0):
        '''DOCSTRING:
        Gets all the black from a picture, returns black and white image
        '''
        # convert BGR to HSV
        frame = cv2.blur(frame,(19, 19))
        ret, frame2 = cv2.threshold(frame, 50, 255, cv2.THRESH_BINARY_INV)
        hsv = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        lower_ref = np.array([0, 0, 100])
        upper_ref = np.array([179, 255, 255])

        # Threshold the SSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_ref, upper_ref)
        mask = 255 - mask

        if showme != 0:
            self.show_image(viewsilhouette, 'silhouetteview')
        return mask

    def transform_to_grid(self, frame, points):
        '''DOCSTRING:
        Input a picture and the reference points that should be the new picture
        returns the new picture
        '''
        zoomed_in = np.float32([[0,0],[0,self.height],[self.width,self.height],[self.width,0]])
        N = cv2.getPerspectiveTransform(points, zoomed_in)
        dst2 = cv2.warpPerspective(frame,N,(self.width,self.height))
        return dst2

    def determine_layout(self, board):
        '''DOCSTRING:
        takes in a nicely gridded board
        returns a list of lists of the status of each space on the board
        Starts in upper left corner of board and goes down
        '''
        layout = []
        for i in range(self.numcols):
            layout.append(['']*self.numrows)
        row = 0
        col = 0

        #iterates through each space
        for y in range(self.numcols):
            for x in range(self.numrows):
                #define the area that space occupies in the picture
                this_tile = board[row*100:(row+1)*100, col*100:(col+1)*100]
                color = self.find_ball_color(this_tile)#,  showme = 1)
                layout[col][row] = color
                row += 1
            row = 0
            col +=1
        return layout

    def find_ball_color(self, space, showme=0):
        '''DOCSTRING:
        Inputs a square of the image representing a space
        Outputs the color of the ball or 'nothing' if none are found
        '''
        #Change to hsv
        hsv = cv2.cvtColor(space, cv2.COLOR_BGR2HSV)
        color = 'Nothing'

        #threshold upper and lower bounds of colors in hsv
        lower_orange = np.array([5, 50, 30])
        upper_orange = np.array([15, 255, 255])
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([105, 255, 255])

        #Create mask for orange threshold and determine if there are any
        #contours large enough to be a ball
        Omask = cv2.inRange(hsv, lower_orange, upper_orange)
        Ores = cv2.bitwise_and(space, space, mask=Omask)
        Oresgrey = cv2.cvtColor(Ores, cv2.COLOR_BGR2GRAY)
        __, contoursO, __ = cv2.findContours(Oresgrey,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contoursO:
            if cv2.contourArea(cnt) > 1500:
                color = 'Orange'

        #Same for blue
        Bmask = cv2.inRange(hsv, lower_blue, upper_blue)
        Bres = cv2.bitwise_and(space, space, mask=Bmask)
        Bresgrey = cv2.cvtColor(Bres, cv2.COLOR_BGR2GRAY)
        __, contoursB, __ = cv2.findContours(Bresgrey,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contoursB:
            if cv2.contourArea(cnt) > 1500:
                color = 'Blue'

        #Displays the square and what color it thinks it found
        if showme != 0:
            self.show_image(Ores, 'Orange')
            self.show_image(Bres, 'Blue')
            print("color found", color)
        return color

    def show_image(self, frame, name='frame'):
        '''DOCSTRING:
        Input an image to show it, hit escape to exit.
        Optional window name.
        '''
        while(True):
            cv2.imshow(name, frame)
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                cv2.destroyAllWindows()
                break
            elif k == ord('s'):
                cv2.imwrite('thisthing.png', frame)

    def detect_corners(self, board, showme = 0):
        '''DOCSTRING:
        Inputs a view of the board slightly bigger than its contour
        Returns the positions of the corners of the board
        for a perspective transform.
        '''
        #Convert to grayscale
        gray = cv2.cvtColor(board, cv2.COLOR_BGR2GRAY)
        boardht, boardwd, __ = board.shape

        #Corner detection (#possible points, quality, smallest distance apart)
        corners = cv2.goodFeaturesToTrack(gray, 300, .01, 12)
        corners = np.int0(corners)
        directions = ['nw', 'sw', 'se', 'ne']
        closest_points = dict()

        #go through each detected corner, determine if it is close to the edge
        for i in corners:
            quadrant = ''
            x, y = i.ravel()
            if showme != 0:
                cv2.circle(board, (x, y), 3, (0, 0, 255), -1)
            if y<100:
                ry = y
                quadrant = quadrant + 'n'
            elif y > boardht-100:
                ry = boardht-y
                quadrant = quadrant + 's'
            if x<100:
                rx = x
                quadrant = quadrant + 'w'
            elif x > boardwd-100:
                rx = boardwd-x
                quadrant = quadrant + 'e'
            #if it is close to 2 edges, it might be the corner of the board
            #Finds the closest corner to the corners of the image
            if len(quadrant)==2:
                if closest_points.has_key(quadrant):
                    oldsize = closest_points[quadrant][1]
                    newsize = np.fabs(rx*ry)
                    if newsize < oldsize:
                        closest_points[quadrant] = [(x, y), np.fabs(rx*ry)]
                else:
                    closest_points[quadrant] = [(x, y), np.fabs(rx*ry)]
            #print(closest_points)

        actual_corners = []
        for point in directions:
            try:
                coordinates = closest_points[point][0]
                cv2.circle(board, coordinates, 7, 255, -1)
                actual_corners.append(list(coordinates))
            except KeyError as err:
                print 'I cannot find a corner in this section:', point

        if showme != 0:
            self.show_image(board, 'corners')

        if len(actual_corners) == 4:
            return np.float32(actual_corners)
        else:
            self.show_image(board, 'corners')

if __name__ =='__main__':
    detect = DetectConnectFour(7, 6)
    detect.run()
