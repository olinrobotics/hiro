#!/usr/bin/env python
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String
import time

class Game:
	def __init__(self):
		rospy.init_node("game_master", anonymous=True)
		self.pub = rospy.Publisher('arm_cmd', String, queue_size=10)
		self.do_next = True
		self.game_running = True
		self.commands = [("E: 13000", "H: -100", "W: 200", "S: 9500"), ("E: 11000", 'placeholder'),
                    ("S: 10500", "E: 9750"), ("S: 11700", "E: 7000"),
                    ("S: 6000", "W: 5000", "S: 8000", "H: 300", "H: 0"),
                    ("W: 0", 'placeholder')]

		self.pub.publish("data: set_speed:: 3000")
		self.waiting_for_next = True

		self.next_move(self.commands[0])
		self.turn_num = 1

		self.ready_to_play()
		self.sub = rospy.Subscriber('whos_turn', String, self.turn_callback, queue_size=None)

	def ready_to_play(self):
		time.sleep(5)

	def turn_callback(self, data):
		print data
		if self.waiting_for_next and data.data == "edwin":
			if self.knocked_over:
				self.waiting_for_next = False
				self.turn_num += 1
				if self.turn_num < len(self.commands):
					self.next_move(self.commands[self.turn_num])
				else:
					self.game_running = False
				self.knocked_over = False
		if data.data == "user":
			self.knocked_over = True

	# def play_game(self):
	# 	if self.do_next == True:
	# 		time.sleep(2)
	# 		self.turn_num += 1
	# 		if self.turn_num < len(self.commands):
	# 			self.next_move(self.commands[self.turn_num])
	# 		else:
	# 			self.game_running = False
	# 		self.do_next = False

	def next_move(self, command):
		time.sleep(5)
		print "sending command ", command
		for tcmd in command:
			cmd = tcmd.split(": ")
			print cmd
			if cmd[0] == "E":
				self.pub.publish("data: rotate_elbow:: " + cmd[1])
			elif cmd[0] == "S":
				self.pub.publish("data: rotate_shoulder:: " + cmd[1])
			elif cmd[0] == "W":
				self.pub.publish("data: rotate_waist:: " + cmd[1])
			elif cmd[0] == "H":
				self.pub.publish("data: rotate_hand:: " + cmd[1])
		self.waiting_for_next = True

	def run(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			r.sleep()

if __name__ == '__main__':
	gm = Game()
	gm.run()
