#!/usr/bin/env python
import rospy
import cv2
import cv2.cv as cv
import numpy as np
import random
import time
import operator
import math
import itertools

from std_msgs.msg import String
from sensor_msgs.msg import Image
from edwin.msg import Edwin_Shape
from cv_bridge import CvBridge, CvBridgeError

def inside_rect(A, B, D, x, y):
	AM_x = (x - A[0])
	AM_y = (y - A[1])

	MB_x = (B[0] - x)
	MB_y = (B[1] - y)

	AB_x = (B[0] - A[0])
	AB_y = (B[1] - A[1])

	AD_x = (D[0] - A[0])
	AD_y = (D[1] - A[1])

	BA_BM = AB_x*MB_x + AB_y*MB_y
	BA_BA = AB_x*AB_x + AB_y*AB_y

	AD_AM = AD_x*AM_x + AD_y*AM_y
	AD_AD = AD_x*AD_x + AD_y*AD_y

	if BA_BM <= 0:
		return False
	elif BA_BM >= BA_BA:
		return False
	elif AD_AM <= 0:
		return False
	elif AD_AM >= AD_AD:
		return False
	else:
		print "TRUE TRUE TRUE"
		return True

class GridDetector:
	def __init__(self, img):
		self.img = img
		self.boxes = []

	def get_distance(self, pt1, pt2):
		return math.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

	def get_pt_x(self, pt1, pt2, d):
		v = (pt1[0]-pt2[0], pt1[1]-pt2[1])
		v_mag = self.get_distance(pt1, pt2)

		u = (v[0]/v_mag, v[1]/v_mag)

		return (int(pt1[0]+d*u[0]), int(pt1[1]+d*u[1]))

	def get_box(self, ref_box, h, id_in):
		pt0 = ref_box[0]
		pt1 = ref_box[1]
		pt2 = self.get_pt_x(ref_box[1], ref_box[2], h)
		pt3 = self.get_pt_x(ref_box[0], ref_box[3], h)

		if id_in == 0:
			return [pt1, pt2, pt3, pt0]
		elif id_in == 1:
			return [pt1, pt2, pt3, pt0]
		elif id_in == 2:
			return [pt1, pt2, pt3, pt0]
		elif id_in == 3:
			return [pt3, pt2, pt1, pt0]
		elif id_in == 4:
			return [pt0, pt1, pt2, pt3]
		elif id_in == 5:
			return [pt0, pt1, pt2, pt3]
		elif id_in == 6:
			return [pt2, pt1, pt0, pt3]
		elif id_in == 7:
			return [pt2, pt1, pt0, pt3]
		elif id_in == 8:
			return [pt2, pt1, pt0, pt3]
		else:
			print "OUT OF BOUNDS"
			return []

	def get_grid(self, box):
		grid_height = int(self.get_distance(box[0], box[1]))
		grid_width = int(self.get_distance(box[1], box[3]))

		box4 = box

		box1 = self.get_box([box[2], box[1], box[0], box[3]], grid_height, 1)
		box3 = self.get_box(box, grid_height, 3)
		box5 = self.get_box([box[3], box[2], box[1], box[0]], grid_height, 5)
		box7 = self.get_box([box[3], box[0], box[1], box[2]], grid_height, 7)

		box0 = self.get_box([box3[2], box3[1], box3[0], box3[3]], grid_height, 0)
		box6 = self.get_box([box3[3], box3[0], box3[1], box3[2]], grid_height, 6)

		box2 = self.get_box([box5[2], box5[1], box5[0], box5[3]], grid_height, 2)
		box8 = self.get_box([box5[3], box5[0], box5[1], box5[2]], grid_height, 8)

		return [box0, box1, box2,
				box3, box4, box5,
				box6, box7, box8]

	def get_center_box(self):
		# img = cv2.imread(im_in)
		# img = im_in
		h, w, ch = self.img.shape

		gray = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)

		corners = cv2.goodFeaturesToTrack(gray,20,0.01,100)
		corners = np.int0(corners)

		dists = {}
		for num, i in enumerate(corners):
			x,y = i.ravel()
			# cv2.circle(img,(x,y),3,255,-1)
			dists[num] = math.sqrt((x-(w/2))**2 + (y-(h/2))**2)

		sorted_dists = sorted(dists.items(), key=operator.itemgetter(1))
		center_rect = []

		pts_distances = []

		if len(sorted_dists) > 5:
			for i in range(4):
				pts_distances.append(sorted_dists[i])
				center_rect.append(list(corners[pts_distances[i][0]].ravel()))

			rect = cv2.minAreaRect(np.int0(center_rect))

			box = cv2.cv.BoxPoints(rect)
			box = np.int0(box)

			boxes = self.get_grid(box)

			for box in boxes:
				box = np.int0(box)

				# cv2.drawContours(img,[box],0,(0,0,255),2)
			self.boxes = np.int0(boxes)

		return corners

	def run(self, img):
		self.img = img

class GridTester:
	def __init__(self):
		rospy.init_node('grid_locator', anonymous = True)
		self.draw_pub = rospy.Publisher('draw_cmd', Edwin_Shape, queue_size=10)
		self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)


		#Board X and Y positions
		self.b_x = 0
		self.b_y = 4000
		self.b_w = 250

		self.board_msg = Edwin_Shape()
		self.draw_msg = Edwin_Shape()

		#A blank space is represented by 0, an "O" is 1, and "X" is 10, we start with blank board
		self.board =   [0, 0, 0,
						0, 0, 0,
						0, 0, 0]

		self.corners = [0,2,6,8] #indices of the corner locations
		self.sides = [1,3,5,7]
		self.middle = 4

	def img_callback(self, data):
		try:
			self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

	def wait_for_hand(self):
		running = True

		hand_see = False
		hand_gone = False

		while running:
			gray = cv2.cvtColor(self.frame,cv2.COLOR_BGR2GRAY)
			ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

			# noise removal
			kernel = np.ones((5,5),np.uint8)
			opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)

			# sure background area
			sure_bg = cv2.dilate(opening,kernel,iterations=3)
			contours, hierarchy = cv2.findContours(sure_bg,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

			areas = [cv2.contourArea(c) for c in contours]
			print max(areas)

			# Find the index of the largest contour
			cv2.drawContours(self.frame, contours, -1, (0,255,0), 3)

			cv2.imshow("img", self.frame)
			cv2.imshow("thresh", sure_bg)
			c = cv2.waitKey(1)

			# if max(areas) > 10000:
			# 	print "FOUND HAND"
			# else:
			# 	print "NO HAND"
			# # 	hand_see = True
			# else:
			# 	if hand_see:
			# 		hand_gone = True
			# if hand_see and hand_gone:
			# 	print "FINISHED DRAWING"
			# 	hand_see = False
			# 	hand_gone = False

	def field_scan(self):
		time.sleep(5)
		#pick marker off paper
		msg = "data: move_to:: 150, 2400, 1500, 0"
		print "sending: ", msg
		self.arm_pub.publish(msg)
		time.sleep(1)

		new_index = {}

		while self.waiting_for_hand:
			time.sleep(1)

		self.waiting_for_hand = False
		running = True
		while running:
			# ret, self.frame = self.cap.read()

			gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

			height, width, channels = self.frame.shape
			self.image_w = width/3
			self.image_h = height/3

			circles = cv2.HoughCircles(gray, cv.CV_HOUGH_GRADIENT,1, 100, param1=50,param2=30,minRadius=20,maxRadius=50)
			if circles is not None:
				circles = np.uint16(np.around(circles))
				circles = np.round(circles[0, :]).astype("int")

				for i in range(9):
					if self.board[i] == 0:
						for (x, y, r) in circles:
							cv2.circle(self.frame, (x, y), r, (0, 255, 0), 4)
							if inside_rect(self.image_rectangles[i][0], self.image_rectangles[i][1], self.image_rectangles[i][3], int(x), int(y)):
								try:
									new_index[i] += 1
									print "INSIDE RECT: ", i
								except KeyError:
									new_index[i] = 1
									print "INSIDE NEW RECT: ", i

				for key in new_index.keys():
					if new_index[key] > 10:
						self.board[key] = 1
						running = False

				print self.board

			#if we detect no circles, exit loop
			else:
				running = False

			for box in self.image_rectangles:
				box = np.int0(box)
				cv2.drawContours(self.frame,[box],0,(0,0,255),2)
				for i in range(3):
					pt = box[i]
					x = pt[0]
					y = pt[1]
					cv2.circle(self.frame,(x,y),3,255,-1)

			cv2.imshow("img", self.frame)
			c = cv2.waitKey(1)

	def draw_the_board(self):
		print "drawing the board"
		self.board_msg.shape = "board"
		self.board_msg.x = self.b_x
		self.board_msg.y = self.b_y
		#note that Z should be a function of y.
		self.board_msg.z = -400 - int((self.board_msg.y - 2500)/9)
		# self.board_msg.z = -640 - int((self.board_msg.y - 2500)/9)
		self.draw_pub.publish(self.board_msg)
		print self.board_msg
		time.sleep(25)

		#look at grid
		# msg = "data: move_to:: 150, 2400, 1500, 0"
		# print "sending: ", msg
		# self.arm_pub.publish(msg)
		# time.sleep(1)

		#look at grid
		msg = "data: move_to:: 120, 2100, 1800, 0"
		print "sending: ", msg
		self.arm_pub.publish(msg)
		time.sleep(1)


		gd = GridDetector(self.frame)
		self.corners = gd.get_center_box()
		self.image_rectangles = gd.boxes

		for i in range(30):
			gd.run(self.frame)
			self.corners = gd.get_center_box()
			for j in range(9):
				elem = gd.boxes[j]
				self.image_rectangles[j] = (self.image_rectangles[j] + elem)/2

	def find_lines(self):
		gray = cv2.cvtColor(self.frame,cv2.COLOR_BGR2GRAY)
		# gray = np.float32(gray)

		# dst = cv2.cornerHarris(gray,2,3,0.04)
		# dst = cv2.dilate(dst,None)

		# self.frame[dst>0.01*dst.max()]=[0,0,255]

		# im_split = cv2.split(self.frame)
		# flag,b = cv2.threshold(im_split[2],0,255,cv2.THRESH_OTSU)

		# element = cv2.getStructuringElement(cv2.MORPH_CROSS,(1,1))
		# cv2.erode(b,element)

		edges = cv2.Canny(gray,50,150,apertureSize = 3)
		minLineLength = 100
		maxLineGap = 10
		lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength,maxLineGap)
		return lines

	def run(self):
		time.sleep(2)
		self.draw_the_board()
		gd = GridDetector(self.frame)
		self.corners = gd.get_center_box()
		self.image_rectangles = gd.boxes

		for i in range(50):
			gd.run(self.frame)
			self.corners = gd.get_center_box()
			for j in range(9):
				elem = gd.boxes[j]
				self.image_rectangles[j] = (self.image_rectangles[j] + elem)/2

		# # fsc = True
		fsc = False
		# msg = "data: move_to:: 120, 2100, 1800, 0"
		# print "sending: ", msg
		# self.arm_pub.publish(msg)
		# time.sleep(2)

		# while not rospy.is_shutdown():
		# 	height, width, channels = self.frame.shape
		# 	if fsc:
		# 		self.field_scan()
		# 	else:
		# 		for box in self.image_rectangles:
		# 			box = np.int0(box)
		# 			cv2.drawContours(self.frame,[box],0,(0,0,255),2)

		# 			box = self.image_rectangles[5]
		# 			#Identifying each of the three points in a box
		# 			for i in range(3):
		# 				pt = box[i]
		# 				x = pt[0]
		# 				y = pt[1]
		# 				if i == 0:
		# 					cv2.circle(self.frame,(x,y),3,(255, 0, 0),3)
		# 				elif i == 1:
		# 					cv2.circle(self.frame,(x,y),5,(0, 255, 0),3)
		# 				elif i == 2:
		# 					cv2.circle(self.frame,(x,y),10,(0, 0, 255),3)

		# 		for num, i in enumerate(self.corners):
		# 			x,y = i.ravel()
		# 			cv2.circle(self.frame,(x,y),3,255,-1)

		# 		cv2.imshow("img", self.frame)
		# 		c = cv2.waitKey(1)


if __name__ == "__main__":
	gf = GridTester()
	gf.run()