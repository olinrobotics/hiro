#!/usr/bin/env python
import rospy
import math
import numpy as np
from std_msgs.msg import String
import Tkinter as tk
import time

class TttGui:
	def __init__(self, root):
		self.master = root

		self.init_control_fields()
		self.init_pub()

	def ttt_send(self, num):
		msg = str(self.ttt_s.get())
		print "sending: ", msg
		self.pub.publish(msg)
		self.ttt_s.delete(0, 'end')

	def init_control_fields(self):
		self.f1 = tk.Frame(self.master)
		self.f1.pack(side=tk.TOP, fill=tk.X)

		self.ttt_s = tk.Entry(self.f1)
		self.ttt_s.bind('<Return>', self.ttt_send)
		tk.Label(self.f1, text="TTT Msg: ").pack(side=tk.LEFT)
		self.ttt_s.pack()

	def init_pub(self):
		rospy.init_node('ttt_tester', anonymous=True)
		self.pub = rospy.Publisher('grid_status', String, queue_size=10)

if __name__ == '__main__':
	root = tk.Tk()
	TttGui(root)
	root.mainloop()