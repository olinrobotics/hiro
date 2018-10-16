#!/usr/bin/env python
import rospy
import math
# import st
import numpy as np
from std_msgs.msg import String
from edwin.msg import *
import time

'''
run robot_write.launch
also run arm_node.py first
'''

def z_calculation(input_y):
	scaler = -735 - int((input_y - 4000)/9.4)
	return scaler


def run():
	rospy.init_node('arm_tester', anonymous=True)
	pub = rospy.Publisher('write_cmd', Edwin_Shape, queue_size=10)
	arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
	time.sleep(2)
	print "starting"

	# while not rospy.is_shutdown():
	msg = Edwin_Shape()


	msg.shape = "123456"
	msg.x = -500
	msg.y = 5700
	msg.z = -835
	pub.publish(msg)
	time.sleep(5)

	msg.shape = "7890!?"
	msg.x = -500
	msg.y = 5400
	msg.z = -835
	pub.publish(msg)
	time.sleep(5)
	#
	# msg.shape = "abcdef"
	# msg.x = -500
	# msg.y = 5700
	# msg.z = -790
	# pub.publish(msg)
	# time.sleep(5)
	#
	# msg.shape = "ghijkl"
	# msg.x = -500
	# msg.y = 5400
	# msg.z = -790
	# pub.publish(msg)
	# time.sleep(5)
	#
	# msg.shape = "mnopqr"
	# msg.x = -500
	# msg.y = 5100
	# msg.z = -780
	# pub.publish(msg)
	# time.sleep(5)
	#
	# msg.shape = "stuvwx"
	# msg.x = -500
	# msg.y = 4800
	# msg.z = -770
	# pub.publish(msg)
	# time.sleep(5)
	#
	# msg.shape = "yz"
	# msg.x = -500
	# msg.y = 4500
	# msg.z = -770
	# pub.publish(msg)
	# time.sleep(5)

if __name__ == '__main__':
	run()
