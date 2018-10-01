#!/usr/bin/env python
import rospy
import copy
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

		if len(sorted_dists) > 4:
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

class Game:
	def __init__(self, init=False):
		if init:
			self.ai = True
		else:
			self.ai = False

		self.draw_pub = rospy.Publisher('draw_cmd', Edwin_Shape, queue_size=10)
		self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
		self.behav_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)

		self.board_msg = Edwin_Shape()
		self.draw_msg = Edwin_Shape()

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)

		self.grid_stat_sub = rospy.Subscriber("grid_status", String , self.grid_callback)

		#Board X and Y positions
		self.b_x = 0
		self.b_y = 4000
		self.b_w = 250

		#Sector centroids
		self.b_centers = {}
		self.b_centers[0] = (self.b_x - 2.25*self.b_w, self.b_y + 2*self.b_w)
		self.b_centers[1] = (self.b_x, self.b_y + 2*self.b_w)
		self.b_centers[2] = (self.b_x + 2*self.b_w, self.b_y + 2*self.b_w)

		self.b_centers[3] = (self.b_x - 2*self.b_w, self.b_y)
		self.b_centers[4] = (self.b_x, self.b_y)
		self.b_centers[5] = (self.b_x + 2*self.b_w, self.b_y)

		self.b_centers[6] = (self.b_x - 2*self.b_w, self.b_y - 1.5*self.b_w)
		self.b_centers[7] = (self.b_x, self.b_y - 1.5*self.b_w)
		self.b_centers[8] = (self.b_x + 2*self.b_w, self.b_y - 1.5*self.b_w)


		#A blank space is represented by 0, an "O" is 1, and "X" is 10, we start with blank board
		self.board =   [0, 0, 0,
						0, 0, 0,
						0, 0, 0]


		self.grid_corners = [0,2,6,8] #indices of the corner locations
		self.grid_sides = [1,3,5,7]
		self.grid_middle = 4
		self.z_depth = -630

		self.difficulty = 10 #0 = easiest, 10 = hardest

	def img_callback(self, data):
		try:
			self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)


	def is_winner(self, board):
		"""
		Computes whether board will win
		"""
		board_sums = [0, 0, 0, 0, 0, 0, 0, 0] #These are the 8 different lines.
		board_sums[0] = board[0] + board[1] + board[2] #Top Horizontal Line
		board_sums[1] = board[3] + board[4] + board[5] #Middle Horizontal Line
		board_sums[2] = board[6] + board[7] + board[8] #Bottom Horizontal Line

		board_sums[3] = board[0] + board[3] + board[6] #Left Vertical Line
		board_sums[4] = board[1] + board[4] + board[7] #Middle Vertical Line
		board_sums[5] = board[2] + board[5] + board[8] #Right Vertical Line

		board_sums[6] = board[0] + board[4] + board[8] #LR Diagonal Line
		board_sums[7] = board[2] + board[4] + board[6] #RL Diagonal Line

		#There are 4 cases - NoWin, EdWin, Player Win and Tie, Tie is accounted for
		#in a piece of code further down.
		for i in range(8):
			b_sum = board_sums[i]
			if b_sum == 30:
				return (1, i)
			elif b_sum == 3:
				return (2, i)
		return (None, None)

	def is_free(self, board, index):
	 	#Function is to check whether a space is occupied or not.
	 	if board[index] == 0:
	 		return True
	 	else:
	 		return False

	def is_board_full(self):
	 	for i in range(9):
	 		if self.is_free(self.board, i):
	 			return False
	 	return True

	def next_move(self):
		#Returns index of next move
		#Checks if Edwin can win on this move

		legal_moves = []
		corner_moves = []
		side_moves = []
		difficulty_rand = random.randint(0,10)

		#if randomly generated number is > difficulty, make a random move
		if difficulty_rand > self.difficulty:
			print "PLAYING RANDOM MOVE"
			board_copy = copy.deepcopy(self.board)
			for i in range(9):
				if self.is_free(board_copy, i):
					legal_moves.append(i)
			return random.choice(legal_moves)

		else:
			#Checks if Edwin wins this turn
			for i in range(9): #don't use len(self.board), it's a numpy array
				board_copy = copy.deepcopy(self.board)
				if self.is_free(board_copy, i):
					board_copy[i] = 10
					if self.is_winner(board_copy)[0] == 1:
						print "WINS"
						return i

			#Checks if player can win the next turn
			for i in range(9):
				board_copy = copy.deepcopy(self.board)
				if self.is_free(board_copy, i):
					board_copy[i] = 1
					if self.is_winner(board_copy)[0] == 2:
						return i

			#Otherwise, prioritizes grabbing corners.
			for i in range(4):
				board_copy = copy.deepcopy(self.board)
				if self.is_free(board_copy, self.grid_corners[i]):
					corner_moves.append(self.grid_corners[i])
			if len(corner_moves) != 0:
				return random.choice(corner_moves)

			#Otherwise, get the middle.
			if self.is_free(board_copy, self.grid_middle):
				board_copy = copy.deepcopy(self.board)
				return self.grid_middle

			#Otherwise, get a side.\
			for i in range(4):
				board_copy = copy.deepcopy(self.board)
				if self.is_free(board_copy, self.grid_sides[i]):
					side_moves.append(self.grid_sides[i])
			if len(side_moves) != 0:
				return random.choice(side_moves)

			return "TIE"

	def manual_field_scan(self):
		#TODO: Update board with current values
		print self.board
		for i in range(9):
			if self.is_free(self.board, i):
				val_in = int(raw_input("Value at "+str(i)+": "))
				self.board[i] = val_in
				if val_in == 1:
					return

	def wait_for_hand(self):
		gray = cv2.cvtColor(self.frame,cv2.COLOR_BGR2GRAY)
		ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

		# noise removal
		kernel = np.ones((5, 5),np.uint8)
		opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)

		# sure background area
		sure_bg = cv2.dilate(opening,kernel,iterations=3)

		contours, hierarchy = cv2.findContours(sure_bg,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
 		cv2.drawContours(self.frame, contours, -1, (0,255,0), 3)
		# Find the index of the largest contour
		areas = [cv2.contourArea(c) for c in contours]

		# gesture_select = random.randint(0, 10000)
		# #there's a one in 5000 chance that edwin will get bored and start looking around
		# if gesture_select == 1:
		# 	self.behav_pub.publish("random")
		# 	time.sleep(5)
		# 	self.arm_pub.publish("data: R_ttt")
		# 	time.sleep(2)
		# 	self.arm_pub.publish("data: R_ttt")

		# cv2.imshow("bg", gray)
		# cv2.imshow("bg2", sure_bg)
		# cv2.imshow("frame", self.frame)
		# c = cv2.waitKey(1)

		if len(areas) == 0:
			areas = [0]

		#TODO: Fix so this isn't necessary. Why is thresh working across half the screen?
		if max(areas) > 100000:
			return False
		elif max(areas) > 10000:
			print "LEN: ", len(areas)
			print "AREA: ", max(areas)
			print "FOUND HAND"
			return True
		else:
			return False

	def field_scan(self):
		time.sleep(5)

		msg = "data: R_ttt"
		print "sending: ", msg
		self.arm_pub.publish(msg)
		time.sleep(3)

		msg = "data: R_ttt"
		print "sending: ", msg
		self.arm_pub.publish(msg)
		time.sleep(3)

		new_index = {}
		start_user_turn = time.time()

		#blocks field_scan until hand is out of the way
		running = True
		while running:
			#if Edwin detects a large contour, he won't look for circles
			if self.wait_for_hand():
				continue

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

				# print self.board

#			if we detect no circles for 15 seconds, publish impatient gesture
			# if int(time.time() - start_user_turn) > 10:
			# 	start_user_turn = time.time()
			# 	self.behav_pub.publish("impatient")
			# 	time.sleep(4)
			# 	self.arm_pub.publish("data: R_ttt")
			# 	time.sleep(2)
			# 	self.arm_pub.publish("data: R_ttt")
			# 	time.sleep(20)

			for box in self.image_rectangles:
				box = np.int0(box)
				cv2.drawContours(self.frame,[box],0,(0,0,255),2)
				for i in range(3):
					pt = box[i]
					x = pt[0]
					y = pt[1]
					cv2.circle(self.frame,(x,y),3,255,-1)

			for num, i in enumerate(self.corners):
					x,y = i.ravel()
					cv2.circle(self.frame,(x,y),3,255,-1)

			cv2.imshow("img", self.frame)
			c = cv2.waitKey(1)


	def draw_the_board(self):
		print "drawing the board"
		self.board_msg.shape = "board"
		self.board_msg.x = self.b_x
		self.board_msg.y = self.b_y
		#note that Z should be a function of y.
		# self.board_msg.z = -500 - int((self.board_msg.y - 2500)/9)
		# self.board_msg.z = self.z_depth - int((self.board_msg.y - 2500)/9)
		self.board_msg.z = self.z_calculation(self.board_msg.y)
		self.draw_pub.publish(self.board_msg)
		print self.board_msg
		time.sleep(30)

		msg = "data: R_ttt"
		print "sending: ", msg
		self.arm_pub.publish(msg)
		time.sleep(3)

		msg = "data: R_ttt"
		print "sending: ", msg
		self.arm_pub.publish(msg)
		time.sleep(3)

		# print "DRAW IN GUIDES IF NECESSARY"
		# time.sleep(5)
		# print "SEND OK COMMAND TO CONTINUE"

		gd = GridDetector(self.frame)
		self.corners = gd.get_center_box()
		self.image_rectangles = gd.boxes

		for i in range(50):
			gd.run(self.frame)
			self.corners = gd.get_center_box()
			for j in range(9):
				elem = gd.boxes[j]
				self.image_rectangles[j] = (self.image_rectangles[j] + elem)/2

	def grid_callback(self, msg):
		print "GRID INFO IS: ", msg.data
		data = msg.data
		if "ok" in msg.data:
			print "OK FOUND"
			self.id = False
		elif "re" in msg.data:
			print "RECALIB FOUND"
			gd = GridDetector(self.frame)
			self.corners = gd.get_center_box()
			self.image_rectangles = gd.boxes

			for i in range(100):
				gd.run(self.frame)
				self.corners = gd.get_center_box()
				for j in range(9):
					elem = gd.boxes[j]
					self.image_rectangles[j] = (self.image_rectangles[j] + elem)/2

	def id_img(self):
		self.calib_grid = True
		while self.calib_grid:
			for box in self.image_rectangles:
				box = np.int0(box)
				cv2.drawContours(self.frame,[box],0,(0,0,255),2)
				for i in range(3):
					pt = box[i]
					x = pt[0]
					y = pt[1]
					cv2.circle(self.frame,(x,y),3,255,-1)

			for num, i in enumerate(self.corners):
					x,y = i.ravel()
					cv2.circle(self.frame,(x,y),3,255,-1)

			cv2.imshow("img", self.frame)
			c = cv2.waitKey(1)
			self.calib_grid = False

	def z_calculation(self, input_y):
		# scaler = int((input_y - 2500)/9.4)
		return self.z_depth

	def edwin_move(self, index):
		#edwin moves to desired location and draws
		print "MOVING TO: ", index
		center = self.b_centers[index]
		self.draw_msg.shape = "x"
		self.draw_msg.x = center[0]
		self.draw_msg.y = center[1]
		#note that Z should be a function of y.
		# self.draw_msg.z = self.z_depth - ((self.draw_msg.y - 2500)/9)
		# self.draw_msg.z = self.z_depth - int((self.draw_msg.y - 2500)/9.4)
		if index < 3:
		    self.draw_msg.z = self.z_depth - 5
		elif index < 6:
		    self.draw_msg.z = self.z_depth
		else:
		    self.draw_msg.z = self.z_depth + 20

		self.draw_pub.publish(self.draw_msg)

	 	self.board[index] = 10
	 	time.sleep(10)

	def draw_win_line(self, win_line):
		#TODO: Put this in arm_draw
		msg = "data: R_ttt"
		print "sending: ", msg
		self.arm_pub.publish(msg)
		time.sleep(2)

		msg = "data: R_ttt"
		print "sending: ", msg
		self.arm_pub.publish(msg)
		time.sleep(2)

		print "DRAWING WIN LINE: ", win_line

		if win_line == 0:
			line = [self.b_centers[0], self.b_centers[2]]
		elif win_line == 1:
			line = [self.b_centers[3], self.b_centers[5]]
		elif win_line == 2:
			line = [self.b_centers[6], self.b_centers[8]]
		elif win_line == 3:
			line = [self.b_centers[0], self.b_centers[6]]
		elif win_line == 4:
			line = [self.b_centers[1], self.b_centers[7]]
		elif win_line == 5:
			line = [self.b_centers[2], self.b_centers[8]]
		elif win_line == 6:
			line = [self.b_centers[0], self.b_centers[8]]
		elif win_line == 7:
			line = [self.b_centers[2], self.b_centers[6]]

		# z = self.z_calculation(int(line[0][1]))
		# z = self.z_depth - int((self.draw_msg.y - 2500)/9.4)
		z = self.z_depth

		#getting into position
		motions = ["data: move_to:: " + str(int(line[0][0])) + ", " + str(int(line[0][1])) + ", " + str(z+250)+ ", " + str(0),
		"data: rotate_hand:: " + str(200),
		"data: rotate_wrist:: " + str(1000)]

		for motion in motions:
			print "sending: ", motion
			self.arm_pub.publish(motion)
			time.sleep(1)

		#pick marker off paper
		msg = "data: move_to:: " + str(int(line[0][0])) + ", " + str(int(line[0][1])) + ", " + str(z+250) + ", " +str(0)
		print "sending: ", msg
		self.arm_pub.publish(msg)
		time.sleep(1)

		msg = "data: move_to:: " + str(int(line[0][0])) + ", " + str(int(line[0][1])) + ", " + str(z) + ", " +str(0)
		print "sending: ", msg
		self.arm_pub.publish(msg)
		time.sleep(1)

		msg = "data: move_to:: " + str(int(line[1][0])) + ", " + str(int(line[1][1])) + ", " + str(z) + ", " +str(0)
		print "sending: ", msg
		self.arm_pub.publish(msg)
		time.sleep(1)

		#pick marker off paper
		msg = "data: move_to:: " + str(int(line[1][0])) + ", " + str(int(line[1][1])) + ", " + str(z+250) + ", " +str(0)
		print "sending: ", msg
		self.arm_pub.publish(msg)
		time.sleep(1)

	def ai_move(self, index):
		#edwin moves to desired location and draws
		print "MOVING TO: ", index
		center = self.b_centers[index]
		self.draw_msg.shape = "square"
		self.draw_msg.x = center[0]
		self.draw_msg.y = center[1]
		#note that Z should be a function of y.
		self.draw_msg.z = self.z_calculation(self.draw_msg.y)
		# self.draw_msg.z = self.z_depth - ((self.draw_msg.y - 2500)/10)
		self.draw_pub.publish(self.draw_msg)

	 	self.board[index] = 1
	 	time.sleep(10)

	def run(self):
	 	time.sleep(5)
	 	print "running"

		self.arm_pub.publish("data: set_speed:: 1000")
		print "set speed to 1000"
		time.sleep(1)

		self.arm_pub.publish("data: set_accel:: 100")
		print "set accel to 100"
		time.sleep(1)

		self.z_depth = -742
		self.draw_the_board()

		turn = 0

		if not self.ai:
			gd = GridDetector(self.frame)
			self.corners = gd.get_center_box()
			self.image_rectangles = gd.boxes

			for i in range(50):
				gd.run(self.frame)
				self.corners = gd.get_center_box()
				for j in range(9):
					elem = gd.boxes[j]
					self.image_rectangles[j] = (self.image_rectangles[j] + elem)/2

			self.id = True
			self.id_img()

			#Player is O: 1
			#Edwin is X: 10
			if turn == 0:
				print "YOUR TURN"
				self.behav_pub.publish("nudge")
				time.sleep(2)

		running = True

		self.behav_pub.publish("R_ttt")
		time.sleep(1)
		while running:
 	 		if turn == 0: #Player turn.
	 			if self.ai:
	 				ai_next_move_ind = self.next_move()
	 				self.ai_move(ai_next_move_ind)
	 			else:
	 				self.field_scan()

				winner = self.is_winner(self.board)
	 			if winner[0] == 2: #Checks if the player made a winning move.
	 				print "PLAYER WINS"
					time.sleep(10)
					# self.draw_win_line(winner[1])
	 				self.behav_pub.publish("sad")
	 				time.sleep(1)
	 				running = False
	 			turn = 1
	 			print "DETECTED CIRCLE, WAITING FOR TURN"
	 			time.sleep(5)
	 			continue

	 		elif turn == 1: #Edwin's turn
	 			next_move_ind = self.next_move()
	 			if next_move_ind == "TIE":
	 				print "IT'S A TIE"
					time.sleep(10)
	 				self.behav_pub.publish("pout")
	 				running = False
	 				continue
	 			self.edwin_move(next_move_ind)
	 			winner = self.is_winner(self.board)
	 			if winner[0] == 1:
	 				print "EDWIN WINS"
					time.sleep(10)
					# 	self.draw_win_line(winner[1])
	 				self.behav_pub.publish("gloat")
	 				time.sleep(1)
	 				running = False
	 			turn = 0

		cv2.destroyAllWindows()
		print "Finished with TTT, hope you enjoyed :)"

if __name__ == '__main__':
	rospy.init_node('ttt_gamemaster', anonymous = True)
	gm = Game()
	gm.run()
