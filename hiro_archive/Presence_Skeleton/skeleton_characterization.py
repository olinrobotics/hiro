#!/usr/bin/env python
import rospy
import rospkg

import math
import time
import csv
import numpy as np
from sklearn.neighbors import KNeighborsClassifier
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
'''
from mpl_toolkits.mplot3d import Axes3D
plt.switch_backend("TkAgg")
'''

from std_msgs.msg import String,Int32MultiArray
from edwin.msg import Bones

def move_to_origin(body):
	'''
	Move the skeleton so that the neck is on the origin
	Returns the new skeleton array
	'''
	new_body = body
	dx = float(body[3])
	dy = float(body[4])
	dz = float(body[5])
	for i in range(len(body)):
		if i%3 == 0:
			new_body[i] = float(body[i]) - dx
		elif i%3 ==1:
			new_body[i] = float(body[i]) - dy
		elif i%3 ==2:
			new_body[i]= float(body[i]) - dz
	return new_body

def find_max(d):
	#Find the key in d that has the biggest value
	m = max(d.values())
	return [k for k,v in d.items() if v==m][0]

class SkeletonDetect:
	def __init__(self):
		#Initialize ROS nodes
		rospy.init_node('skeleton_detect',anonymous=True)
		self.detect_pub = rospy.Publisher('/skeleton_detect',String,queue_size = 10)
		rospy.Subscriber('/all_control', String, self.control_callback, queue_size=10)
		rospy.Subscriber('/skeleton', Bones, self.skeleton_callback)

		#Initialize class variables
		self.time_interval= 0
		self.test_data = []
		self.knn = KNeighborsClassifier(n_neighbors = 3)
		self.old_time = 0
		self.is_detecting = False
		self.gesture = dict()

		#Dictionary for detecting moving gestures
		self.moving = {"disco1":False,"disco2":False,"bow1":False,"bow2":False}

		rospack = rospkg.RosPack()
		PACKAGE_PATH = rospack.get_path("edwin")
		filepath = PACKAGE_PATH + "/scripts/sight/skeleton.csv"
		#Read training data

		with open(filepath, 'r') as f:
			reader = csv.reader(f)
			self.X_data = [col[1:] for col in reader][1:]
		with open(filepath,'r') as f:
			reader = csv.reader(f)
			self.Y_data = [col[0] for col in reader][1:]

		print("Gesture Detection is running")

	def control_callback(self,data):
		"""
		Parse /all_control message
		"""
		if "gesture_detect:stop" in data.data:
			self.is_detecting = False
		else:
			split_data = data.data.split(" ")
			self.time_interval= int(split_data[1])
			self.old_time = time.localtime().tm_sec
			self.is_detecting = True

	def skeleton_callback(self,data):
		"""
		Parse /skeleton message from skeleton.py
		"""
		self.test_data = []
		self.test_data.append(data.h.x)
		self.test_data.append(data.h.y)
		self.test_data.append(data.h.z)
		self.test_data.append(data.n.x)
		self.test_data.append(data.n.y)
		self.test_data.append(data.n.z)
		self.test_data.append(data.t.x)
		self.test_data.append(data.t.y)
		self.test_data.append(data.t.z)
		self.test_data.append(data.rs.x)
		self.test_data.append(data.rs.y)
		self.test_data.append(data.rs.z)
		self.test_data.append(data.re.x)
		self.test_data.append(data.re.y)
		self.test_data.append(data.re.z)
		self.test_data.append(data.rh.x)
		self.test_data.append(data.rh.y)
		self.test_data.append(data.rh.z)
		self.test_data.append(data.ls.x)
		self.test_data.append(data.ls.y)
		self.test_data.append(data.ls.z)
		self.test_data.append(data.le.x)
		self.test_data.append(data.le.y)
		self.test_data.append(data.le.z)
		self.test_data.append(data.lh.x)
		self.test_data.append(data.lh.y)
		self.test_data.append(data.lh.z)
		self.test_data = move_to_origin(self.test_data)

	def train_data_processing(self):
		"""
		Preprocess training data
		"""
		for i in range(len(self.X_data)):
			self.X_data[i] = move_to_origin(self.X_data[i])

	def skeleton_detect_train(self):
		"""
		K-nearest neighbor machine learning code
		"""
		self.knn.fit(self.X_data,self.Y_data)
		print("Finish Training")

		#Testing accuracy

		"""
		X_train, X_test, Y_train, Y_test = train_test_split(self.X_data, self.Y_data, test_size=0.2, random_state=42)
		self.knn.fit(X_train,Y_train)
		print(self.knn.score(X_test,Y_test))
		"""

	def skeleton_detect_test(self):
		"""
		Predict gesture from input skeleton
		"""
		try:
			result = self.knn.predict([self.test_data])[0]
			print(result)

			#Set Key Frame to true
			self.moving['disco1'] = self.moving['disco1'] or result == "disco1"
			self.moving['disco2'] = self.moving['disco2'] or result == "disco2"
			self.moving['bow1'] = self.moving['bow1'] or result == "bow1"
			self.moving['bow2'] = self.moving['bow2'] or result == "bow2"

			#Add predicted result to current gesture dictionary
			if result != "disco1" and result != "disco2" and result != "bow1" and result != "bow2":
				self.gesture[result] = self.gesture.get(result,0) + 1
			elif result == "disco1" and not self.moving['disco2']:
				self.gesture['wave'] = self.gesture.get('wave',0) + 1
			elif (result == "disco1" or result == "disco2") and self.moving['disco1'] and self.moving['disco2']:
				self.gesture['disco'] = self.gesture.get('disco',0) + 1
			elif (result == "bow1" or result == "bow2") and self.moving['bow1'] and self.moving['bow2']:
				self.gesture['bow'] = self.gesture.get('bow',0) + 1
		except:
			pass

	def publish_result(self):
		"""
		Publish the predicted gesture after a time interval
		"""
		if time.localtime().tm_sec ==  (self.old_time + self.time_interval) % 60 :
			try:
				self.detect_pub.publish(find_max(self.gesture))
				self.gesture = dict()
				self.moving = {"disco1":False,"disco2":False,"bow1":False,"bow2":False}

				# stop detecting for 1 second to make sure there is only one response published
				time.sleep(1)
			except:
				pass

	def run(self):
		r = rospy.Rate(10)
		self.train_data_processing()
		self.skeleton_detect_train()

		while not rospy.is_shutdown():
			if self.is_detecting:
				self.skeleton_detect_test()
				self.publish_result()
			r.sleep()

if __name__ == '__main__':
	sd = SkeletonDetect()
	sd.run()
