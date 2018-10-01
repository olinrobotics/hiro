#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from edwin.msg import *
import time
import subprocess

def sounds():
	pub = rospy.Publisher('edwin_sounds', String, queue_size=10)
	rospy.init_node('edwin_sounds_publisher', anonymous=True)
	pub.publish(raw_input("What sound do you desire?"))
	time.sleep(1)

if __name__ == '__main__':

	sounds()