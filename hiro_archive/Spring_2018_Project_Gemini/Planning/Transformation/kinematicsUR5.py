#!/usr/bin/env python

"""
Benjamin Ziemann
benjamin.ziemann@students.olin.edu

A coordinate frame transformation and forward kinematic model
for the UR5 robotic arm and endefactor
used in Olin's Robotics Lab for Project Gemini

Dependencies: numpy

To run:
Create an instance of the class
Use .fkine with a list of angles in degrees

TODO:
Implement inverse Kinematics
Understand rotational part of the matrix
"""

import numpy as np
import math
from ziemannMath import rotateX, rotateY, rotateZ, transX, transY, transZ
"""
A coordinate frame transformation and forward kinematic model
for the UR5 robotic arm and endefactor
used in Olin's Robotics Lab for Project Gemini
"""

#Nice option to stop scientific notation
np.set_printoptions(suppress=True)

class coordinateFrames:

	def __init__(self):
		#All received angle measurements are in degrees and later converted to radians
		#Base, shoulder, elbow, w1, w2, w3
		self.jointPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

		#DH Paramers - theta, a, alpha, d
		self.dh =  [[self.jointPos[0],	0.0, 		math.pi/2,	0.089159],
				 	[self.jointPos[1],	-0.425, 	0.0, 		0.0],
				 	[self.jointPos[2],	-0.39225, 	0.0,		0.0],
				 	[self.jointPos[3],	0.0, 		math.pi/2,	0.10915],
				 	[self.jointPos[4],	0.0, 		-math.pi/2,	0.09465],
				 	[self.jointPos[5],	0.0, 		0.0,		0.0823]]
		
		#All distance measurements are in mm
		#Initial position
		self.position = np.transpose(np.matrix([0.0, 0.0, 0.0, 1.0]))


	def updateDH(self, angles):
		"""
		Update DH parameters based on new angless
		"""
		for i in range(len(angles)):
			self.jointPos[i] = np.radians(angles[i])

		for i in range(len(self.jointPos)):
			self.dh[i][0] = self.jointPos[i]


	def fkine(self, angles):
		"""
		Function that takes in angle measurements and converts it to an X, Y, Z positon for the endefactor
		"""
		self.updateDH(angles)

		#Link 1
		T_01 = np.matrix(transZ(self.dh[0][3]).dot(rotateZ(self.dh[0][0])).dot(transX(self.dh[0][1])).dot(rotateX(self.dh[0][2])))

		#Link 2
		T_12 = np.matrix(transZ(self.dh[1][3]).dot(rotateZ(self.dh[1][0])).dot(transX(self.dh[1][1])).dot(rotateX(self.dh[1][2])))
		
		#Link 3
		T_23 = np.matrix(transZ(self.dh[2][3]).dot(rotateZ(self.dh[2][0])).dot(transX(self.dh[2][1])).dot(rotateX(self.dh[2][2])))
		
		#Link 4
		T_34 = np.matrix(transZ(self.dh[3][3]).dot(rotateZ(self.dh[3][0])).dot(transX(self.dh[3][1])).dot(rotateX(self.dh[3][2])))
		
		#Link 5
		T_45 = np.matrix(transZ(self.dh[4][3]).dot(rotateZ(self.dh[4][0])).dot(transX(self.dh[4][1])).dot(rotateX(self.dh[4][2])))
		
		#Link 6
		T_56 = np.matrix(transZ(self.dh[5][3]).dot(rotateZ(self.dh[5][0])).dot(transX(self.dh[5][1])).dot(rotateX(self.dh[5][2])))
		

		T_fin = T_01.dot(T_12).dot(T_23).dot(T_34).dot(T_45).dot(T_56)
		print("Original dotting")
		print(T_fin)


		# #Straight up hardcoding end arrays
		# #Link 1
		# print("hardcode")
		# T_01 = np.matrix([[np.cos(self.dh[0][0]),	-np.sin(self.dh[0][0])*np.cos(self.dh[0][2]), np.sin(self.dh[0][0])*np.sin(self.dh[0][2]),  self.dh[0][1]*np.cos(self.dh[0][0])],
		# 		[np.sin(self.dh[0][0]),	np.cos(self.dh[0][0])*np.cos(self.dh[0][2]),  -np.cos(self.dh[0][0])*np.sin(self.dh[0][2]), self.dh[0][1]*np.sin(self.dh[0][0])],
		# 		[0.0,					np.sin(self.dh[0][2]), 						  np.cos(self.dh[0][2]), 						self.dh[0][3]],
		# 		[0.0,					0.0,											0.0,										1.0]])
		
		# #Link 2
		# T_12 = np.matrix([[np.cos(self.dh[1][0]),	-np.sin(self.dh[1][0])*np.cos(self.dh[1][2]), np.sin(self.dh[1][0])*np.sin(self.dh[1][2]),  self.dh[1][1]*np.cos(self.dh[1][0])],
		# 		[np.sin(self.dh[1][0]),	np.cos(self.dh[1][0])*np.cos(self.dh[1][2]),  -np.cos(self.dh[1][0])*np.sin(self.dh[0][2]), self.dh[1][1]*np.sin(self.dh[1][0])],
		# 		[0.0,					np.sin(self.dh[1][2]), 						  np.cos(self.dh[1][2]), 						self.dh[1][3]],
		# 		[0.0,					0.0,											0.0,										1.0]])

		# #Link 3
		# T_23 = np.matrix([[np.cos(self.dh[2][0]),	-np.sin(self.dh[2][0])*np.cos(self.dh[2][2]), np.sin(self.dh[2][0])*np.sin(self.dh[2][2]),  self.dh[2][1]*np.cos(self.dh[2][0])],
		# 		[np.sin(self.dh[2][0]),	np.cos(self.dh[2][0])*np.cos(self.dh[2][2]),  -np.cos(self.dh[2][0])*np.sin(self.dh[2][2]), self.dh[2][1]*np.sin(self.dh[2][0])],
		# 		[0.0,					np.sin(self.dh[2][2]), 						  np.cos(self.dh[2][2]), 						self.dh[2][3]],
		# 		[0.0,					0.0,											0.0,										1.0]])

		# #Link 4
		# T_34 = np.matrix([[np.cos(self.dh[3][0]),	-np.sin(self.dh[3][0])*np.cos(self.dh[3][2]), np.sin(self.dh[3][0])*np.sin(self.dh[3][2]),  self.dh[3][1]*np.cos(self.dh[3][0])],
		# 		[np.sin(self.dh[3][0]),	np.cos(self.dh[3][0])*np.cos(self.dh[3][2]),  -np.cos(self.dh[3][0])*np.sin(self.dh[3][2]), self.dh[3][1]*np.sin(self.dh[3][0])],
		# 		[0.0,					np.sin(self.dh[3][2]), 						  np.cos(self.dh[3][2]), 						self.dh[3][3]],
		# 		[0.0,					0.0,											0.0,										1.0]])

		# #Link 5
		# T_45 = np.matrix([[np.cos(self.dh[4][0]),	-np.sin(self.dh[4][0])*np.cos(self.dh[4][2]), np.sin(self.dh[4][0])*np.sin(self.dh[4][2]),  self.dh[4][1]*np.cos(self.dh[4][0])],
		# 		[np.sin(self.dh[4][0]),	np.cos(self.dh[4][0])*np.cos(self.dh[4][2]),  -np.cos(self.dh[4][0])*np.sin(self.dh[4][2]), self.dh[4][1]*np.sin(self.dh[4][0])],
		# 		[0.0,					np.sin(self.dh[4][2]), 						  np.cos(self.dh[4][2]), 						self.dh[4][3]],
		# 		[0.0,					0.0,											0.0,										1.0]])


		# #Link 6
		# T_56 = np.matrix([[np.cos(self.dh[5][0]),	-np.sin(self.dh[5][0])*np.cos(self.dh[5][2]), np.sin(self.dh[5][0])*np.sin(self.dh[5][2]),  self.dh[5][1]*np.cos(self.dh[5][0])],
		# 		[np.sin(self.dh[5][0]),	np.cos(self.dh[5][0])*np.cos(self.dh[5][2]),  -np.cos(self.dh[5][0])*np.sin(self.dh[5][2]), self.dh[5][1]*np.sin(self.dh[5][0])],
		# 		[0.0,					np.sin(self.dh[5][2]), 						  np.cos(self.dh[5][2]), 						self.dh[5][3]],
		# 		[0.0,					0.0,											0.0,										1.0]])

		# T_fin = T_01.dot(T_12).dot(T_23).dot(T_34).dot(T_45).dot(T_56)
		# print("Written out")
		# print(T_fin)


if __name__ == "__main__":
	cf = coordinateFrames()
	cf.fkine([0.0, -90.0, 0.0, -90.0, 0.0, 0.0])