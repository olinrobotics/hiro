#!/usr/bin/env python
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String
import time

class Drawer:
	def __init__(self):
		rospy.init_node("draw_master", anonymous=True)
		self.pub = rospy.Publisher('arm_cmd', String, queue_size=10)

		self.pub.publish("data: set_speed:: 3000")
		self.ready_to_play()
		self.sub = rospy.Subscriber('draw_me', String, self.draw_callback, queue_size=None)

	def ready_to_play(self):
		time.sleep(5)

	def draw_callback(self, data):
		print data
		if data.data == "square":
			head = "data: move_to:: "
			cmds = ["500, 200, 300, 0", "500, 400, 300, 0", "500, 400, 500, 0", "500, 200, 500, 0", "500, 200, 300, 0"]
			for elem in cmds:
				self.pub.publish(head + elem)
				time.sleep(2)

	def run(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			r.sleep()

if __name__ == '__main__':
	dm = Drawer()
	dm.run()