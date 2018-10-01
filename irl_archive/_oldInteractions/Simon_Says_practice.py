"""
This is the demo script to be integrated with Simon Says main script.

issus_simon_cmd publishes a random command from the simon says dictionary
check_simon_response checks if the player is doing the correct gesture
"""
import rospy
import cv2
import cv2.cv as cv
import numpy as np
import random
import time
import math
from collections import namedtuple

from std_msgs.msg import String
from sensor_msgs.msg import Image
from edwin.msg import Edwin_Shape, Bones
from cv_bridge import CvBridge, CvBridgeError

EDWIN_NAME = "edwin"
USER_NAME = "user"

class Game:
	def __init__(self, max_turns = 15):
		self.say_pub = rospy.Publisher('/say_cmd', String, queue_size = 1)
		self.ctr_pub = rospy.Publisher('/all_control',String, queue_size=10)
		self.res_pub = rospy.Publisher('/display_result',String, queue_size=1)
		rospy.Subscriber("/skeleton_detect", String, self.gest_callback, queue_size = 10)
		self.current_cmd = None
		self.first = True
		self.gesture = ""
		self.max_turns = max_turns
		self.command_dictionary = {}
		self.msg = ""
		self.populate_command_dictionaries()
		self.simonless_gest = None

	def gest_callback(self,data):
		self.gesture = data.data

	def populate_command_dictionaries(self):
		self.command_dictionary["touch_head"] = "Touch your head with left hand"
		self.command_dictionary["rub_tummy"] = "Rub your tummy with both hands"
		self.command_dictionary["high_five"] = "Clap to your left"
		self.command_dictionary["wave"] = "Wave to your right"
		self.command_dictionary["dab"] = "Dab into your right elbow"
		self.command_dictionary["disco"] = "Disco with your right hand"
		self.command_dictionary["bow"] = "Just Bow"
		self.command_dictionary["star"] = "Do a starfish"
		self.command_dictionary["heart"] = "Form a heart with your arms"

	def issue_simon_cmd(self):
		"""
		If Simon is Edwin:
		This function issues a Simon command through voice command.
		"""
		# randomly pick a command from dictionary
		command = random.choice(self.command_dictionary.keys())

		# makes sure that command is not the same twice in a row
		try:
			while self.command_dictionary[command] in self.current_cmd:
				command = random.choice(self.command_dictionary.keys())
		except:
			pass

		#makes sure that first command contains 'simon says'
		if self.first == True:
			self.current_cmd = "simon says, " + self.command_dictionary[command]
			self.first = False
		else:
			self.current_cmd = random.choice(["simon says, ", ""]) + self.command_dictionary[command]
		self.say_pub.publish(self.current_cmd)

		# wait two seconds for simon to speak command
		time.sleep(2)

		# publishes to all_control that tells it to go and wait 4 seconds for detecting gestures
		#if "Wave" in self.current_cmd or "Disco" in self.current_cmd or "Bow" in self.current_cmd or "Hug" in self.current_cmd:
		self.msg = "gesture_detect:go 4"
		self.ctr_pub.publish(self.msg)

	def check_simon_response(self):
		"""If Simon is Edwin:
		This function checks to see if the players have followed Edwin's cmd
		"""
		if "simon says" in self.current_cmd:
			command_gest = self.current_cmd.replace("simon says, ","")
			for key,value in self.command_dictionary.items():
				if value == command_gest:
					command_gest = key

			#compares command to the gesture recived from subscriber
			if command_gest  == self.gesture:
				self.res_pub.publish('Good Job!')
				self.simonless_gest = self.gesture
			else:
				self.res_pub.publish('Try Again!')
				self.simonless_gest = self.gesture
		else:
			if self.simonless_gest == self.gesture:
				self.res_pub.publish('Good Job!')
			else:
				self.res_pub.publish('Try Again!')

	def run(self):
		"""Game.py mainloop. Runs for as long as max_turns is defined"""

		time.sleep(2)
		print "running"

		self.simon_ID = EDWIN_NAME

		#two phases of simon says, issue command, check for follower response
		turn_count = 0
		time.sleep(5)
		self.ctr_pub.publish("gesture_detect:go")

		# run the game for max_turns tiems
		while turn_count < self.max_turns:
			turn_count += 1

			#issue command
			self.issue_simon_cmd()
			#if "Wave" in self.current_cmd or "Disco" in self.current_cmd or "Bow" in self.current_cmd or "Hug" in self.current_cmd:

			# wait for player response from subscriber
			time.sleep(4)

			#check for response
			self.check_simon_response()
		self.ctr_pub.publish("gesture_detect:stop")
		print "Finished with Simon Says, hope you enjoyed :)"

if __name__ == '__main__':
	rospy.init_node('ss_gamemaster', anonymous = True)
	gm = Game()
	gm.run()
