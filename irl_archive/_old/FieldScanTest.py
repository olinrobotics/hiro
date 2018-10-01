import cv2
import cv2.cv as cv
import numpy as np
import copy

def inside_rect(rx, ry, w, h, x, y):
	# rx = rect[0]
	# ry = rect[1]
	if rx < x < rx+w and ry < y < ry+h:
		return True
	else:
		return False

def FieldScan():
	# FieldScan returns the digitized 0-8 array as a board.
	#I suppose FieldScan works on the assumption that the human doesn't cheat.
	#Technically, Edwin will look for the grid, and assign each box a number in the
	#array.
	cap = cv2.VideoCapture(1)

	rectangles = [(0, 0), (117, 0), (234, 0),
				(0, 96), (117, 96), (234, 96),
				(0, 192), (117, 192),(234, 192)]

	while True:
		#This program should find the large tic tac toe box,break it down into
		#9 different cells, and then analyze the contents of each cell.
		ret, frame = cap.read()
		height, width, channels = frame.shape
#		print height, width
		imgray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		ret,thresh = cv2.threshold(imgray,115,255,0)
		blur = cv2.blur(thresh, (2,2))

		contours,h = cv2.findContours(blur,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_TC89_L1)
		circles = cv2.HoughCircles(imgray, cv.CV_HOUGH_GRADIENT,1, 100, param1=50,param2=30,minRadius=20,maxRadius=50)

		# #Finds center rectangle of board
		# for cont in contours:
		# 	approx = cv2.approxPolyDP(cont, 0.01*cv2.arcLength(cont, True), True)
		# 	sides = len(approx)
		# 	if sides == 4:
		# 		x,y,w,h = cv2.boundingRect(cont)
		# 		cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,255),2)

		#Finds circles

		w = 117
		h = 96

		board = [0,0,0,
			0,0,0,
			0,0,0]

		if circles is not None:
			circles = np.round(circles[0, :]).astype("int")
			for (x, y, r) in circles:
				cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
				for i in range(9):
					if inside_rect(rectangles[i][0], rectangles[i][1], w, h, int(x), int(y)):
						board[i] = 10
						continue

		for rect in rectangles:
			cv2.rectangle(frame, rect, (rect[0]+w, rect[1]+h), (0,0,255), 2)

		cv2.imshow("camera", frame)
		c = cv2.waitKey(1)

if __name__ == '__main__':
	FieldScan()
