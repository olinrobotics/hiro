#!/usr/bin/env python


"""
Benjamin Ziemann
benjamin.ziemann@students.olin.edu

Adapted homogonous transformation matrix for use with
DH parametersfor use in
Olin Interactive Robotics Lab Project Gemini

Dependencies: numpy
"""

import numpy as np

def rotateX(angle, offset=0):
	"""
	Rotate about the x axis of a coordinate frame

	Angle and offset are in radians
	"""
	return 	np.matrix([[1.0, 	0.0, 					0.0, 					0.0],
						[0.0, 	np.cos(angle+offset),	-np.sin(angle+offset) , 0.0],
						[0.0, 	np.sin(angle+offset),	np.cos(angle+offset),	0.0],
						[0.0, 	0.0, 					0.0, 					 1.0]])

def rotateY(angle, offset=0):
	"""
	Rotate about the y axis of a coordinate frame

	Angle and offset are in radians
	"""
	return 	np.matrix([[np.cos(angle+offset), 		0.0, 	np.sin(angle+offset), 	0.0],
						[0.0, 						1.0,	0.0,	 				0.0],
						[-np.sin(angle+offset), 	0.0,	np.cos(angle+offset),	0.0],
						[0.0, 						0.0, 	0.0, 					1.0]])

def rotateZ(angle, offset=0):
	"""
	Rotate about the z axis of a coordinate frame

	Angle and offset are in radians
	"""
	return 	np.matrix([[np.cos(angle+offset), -np.sin(angle+offset),	0.0, 	0.0],
						[np.sin(angle+offset), 	np.cos(angle+offset),	0.0,	0.0],
						[0.0, 					0.0,					1.0,	0.0],
						[0.0, 					0.0, 					0.0, 	1.0]])

def transX(distance):
	"""
	Translation along the x axis of a coordinate frame

	Distance is in mm
	"""
	return 	np.matrix([[1.0, 	0.0,	0.0, 	distance],
						[0.0, 	1.0,	0.0,	0.0],
						[0.0, 	0.0,	1.0,	0.0],
						[0.0, 	0.0, 	0.0, 	1.0]])

def transY(distance):
	"""
	Translation along the y axis of a coordinate frame

	Distance is in mm
	"""
	return 	np.matrix([[1.0, 	0.0,	0.0, 	0.0],
						[0.0, 	1.0,	0.0,	distance],
						[0.0, 	0.0,	1.0,	0.0],
						[0.0, 	0.0, 	0.0, 	1.0]])

def transZ(distance):
	"""
	Translation along the z axis of a coordinate frame

	Distance is in mm
	"""
	return 	np.matrix([[1.0, 	0.0,	0.0, 	0.0],
						[0.0, 	1.0,	0.0,	0.0],
						[0.0, 	0.0,	1.0,	distance],
						[0.0, 	0.0, 	0.0, 	1.0]])