#!/usr/bin/env python
"""
This is the demo script for the SimonSays module. It requires theses modules to be running:

rosrun edwin arm_node.py
rosrun edwin arm_behaviors.py
rosrun edwin tts_engine.py
rosrun edwin stt_engine.py

roslaunch skeleton_markers markers_from_tf.launch
roscd skeleton_markers
rosrun rviz rviz markers_from_tf.rviz

rosrun edwin skeleton.py
rosrun edwin skeleton_characterization.py
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

"""
This class plays the game of Simon Says in three different ways: by himself, as Simon with a  player,
or as a player with Simon. The basic game is that Simon will tell the player to perform a certain action,
and the player must only perform a movement when Simon adds 'simon says' to the beginning of the sentence.
The player loses if he makes an incorrect move or moves under a command that didn't have the 'simon says' prefix.

The three modes are split up and draw from a parent class that holds general game functionality. The basic three steps in the game for Edwin
regardless of his role is:

1. listen/say a command
2. act/check on response to command
3. continue until a set number of turns or until the player loses
"""

SIMON_NAME = "simon"
PLAYER_NAME = "player"

class SimonSays(object):
	"""
	The master class of the game Simon Says, which holds all the general actions of the Game.py
	To be the parent of the actual game modes:
	1. AutonomousSimon - plays by himself
	2. SimonEdwin - Edwin is the Simon and issues commands to the user
	3. SimonUser - The user if the Simon and tell Edwin what to do
	"""
	def __init__(self, max_turns = 5):
		#init ROS nodes
		rospy.init_node('ss_gamemaster', anonymous = True)

		#ROS subscriber variables
		self.ready_to_listen = False       #stt
		self.heard_cmd = None              #stt
		self.listen_for_fail = None		   #stt
		self.current_body_points = None    #skeleton
		self.body_list = []				   #skeleton
		self.frame = None                  #camera

		#Edwin's current command of interest
		self.current_cmd = None            #descriptive verbal command
		self.current_act = None            #edwin command (not necessarily in arm_cmd form)
		self.previous_cmd = -1		       #prevents the same action from being called twice
		self.arm_act = None				   #edwin arm_node command
		self.simon_or_naw = True		   #whether or not Edwin knows simon says or not
		self.missed_statement = False      #boolean to determine whether something was caught

		#Edwin's history of past movements the user has made
		self.history = []
		self.history_length = 10 #depth of recorded history
		self.threshold = .05  #threshold to determine movement

		#max iterations per game, and variable to keep track of game procession
		self.max_turns = max_turns
		self.no_mistakes = True  # detects if a mistake is made following a cmd; for Edwin or user

		#the dictionaries that contain what Edwin will do based on a Simon command
		self.command_2_speech = {}     	    #specific command whose key points to verbal command
		self.command_2_motion = {}			#specific command whose key points to edwin action
		self.command_dictionary = {}		#finding gestures

		# for gestures
		self.gesture = None
		self.looking = False

		#populate the two dictionaries for when Edwin is Simon and when he is the player
		self.populate_simon_dictionaries()
		self.populate_player_dictionaries()
		self.populate_command_dictionaries()

		#for the image
		self.bridge = CvBridge()

		#for checking actuation in services
		self.status = -1


		#init ROS publishers for actuation and stated speech
		self.behavior_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
		self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
		self.say_pub = rospy.Publisher('edwin_speech_cmd', String, queue_size = 1)

		#init ROS subscribers to camera, heard speech, and skeleton
		self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)
		self.hear_sub = rospy.Subscriber("simon_stt", String, self.hear_callback)
		self.skelesub = rospy.Subscriber("skeleton", Bones, self.skeleton_callback)
		self.status_sub = rospy.Subscriber('/arm_status', String, self.status_callback, queue_size=10)

		self.always_simon = False

	def status_callback(self, data):
		print "arm status callback", data.data
		if data.data == "busy" or data.data == "error":
			self.status = 0
		elif data.data == "free":
			self.status = 1


	def img_callback(self, data):
		try:
			self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print e



	def hear_callback(self, speech):
		if self.ready_to_listen:
			self.heard_cmd = speech.data
		self.listen_for_fail = speech.data
		if ("incorrect" in self.listen_for_fail) or ("wrong" in self.listen_for_fail):
			self.no_mistakes = False

	def gest_callback(self,data):
		if self.looking:
			self.gesture = data

	def skeleton_callback(self, skeleton):
		"""
        Parses out skeleton data

		Skeleton is stored as an object with 15 different variables, one for each skeleton point

		h - head
		n - neck
		t - torso

		rs - right shoulder
		re - right elbow
		rh - right hand

		rp - right hip
		rk - right knee
		rf - right foot

		ls - left shoulder
		le - left elbow
		lh - left hand

		lp - left hip
		lk - left knee
		lf - left foot
        """

		#this variables allows for referencing specific body parts
		self.current_body_points = skeleton

		#this variable is simply for iterating through all parts
		self.body_list = (self.current_body_points.h,
						  self.current_body_points.n,
						  self.current_body_points.t,
						  self.current_body_points.rs,
						  self.current_body_points.re,
						  self.current_body_points.rh,
						  self.current_body_points.rp,
						  self.current_body_points.rk,
						  self.current_body_points.rf,
						  self.current_body_points.ls,
						  self.current_body_points.le,
						  self.current_body_points.lh,
						  self.current_body_points.lp,
						  self.current_body_points.lk,
						  self.current_body_points.lf)
		self.history.append(self.body_list)
		if len(self.history) > self.history_length:
			self.history.pop(0)


	def populate_simon_dictionaries(self):
		"""
		Fill up the Simon command dictionary possible commands
		"""
		self.command_2_speech["heart"] = "make a heart above your head"
		self.command_2_speech["touch_head"] = "touch your head with your left hand"
		self.command_2_speech["rub_tummy"] = "rub your tummy with both hands"
		self.command_2_speech["high_five"] = "give yourself a high five to your left"
		self.command_2_speech["star"] = "make a starfish"

		##NON WORKING GESTURES
		# self.command_2_speech["hug_self"] = "hug yourself"
		self.command_2_speech["wave"] = "wave at the camera with your right hand"
		self.command_2_speech["dab"] = "do the dab to the right"
		self.command_2_speech["disco"] = "do the disco with your right hand"
		self.command_2_speech["bow"] = "bow down to the master"


	def populate_player_dictionaries(self):
		"""
		Fill up the Player command dictionary possible commands
		"""
		self.command_2_motion["heart"] = ["heart"]
		self.command_2_motion["touch head"] = ["touch_head"]
		self.command_2_motion["rub tummy"] = ['rub_tummy']
		self.command_2_motion["high five"] = ['high_five']
		self.command_2_motion["hug yourself"] = ["hug_self"]
		self.command_2_motion["dab"] = ["dab"]
		self.command_2_motion["star"] = ["star"]
		self.command_2_motion["wave"] = ["wave"]
		self.command_2_motion["disco"] = ["disco"]
		self.command_2_motion["bow"] = ["bow"]


	def populate_command_dictionaries(self):
		self.command_dictionary["touch_head"] = "touch_head"
		self.command_dictionary["rub_tummy"] = "rub_tummy"
		self.command_dictionary["high_five"] = "high_five"
		self.command_dictionary["hug_self"] = "hug_self"
		self.command_dictionary["wave"] = "wave"
		self.command_dictionary["dab"] = "dab"
		self.command_dictionary["disco"] = "disco"
		self.command_dictionary["bow"] = "bow"
		self.command_dictionary["star"] = "star"
		self.command_dictionary["heart"] = "heart"


	def check_completion(self):
		"""
		makes sure that actions run in order by waiting for response from service
		"""

		time.sleep(3)
		while self.status == 0:
			pass

	def simon_say_command(self):
		"""
		If Edwin is Simon:

		This function issues a Simon command.
		It is said outloud, so depends on the tts_engine to be running
		"""

		command = random.choice(self.command_2_speech.keys())
		while command == self.previous_cmd or command == "hug_self":
			command = random.choice(self.command_2_speech.keys())
		print "PREV:", self.previous_cmd, "CURRENT", command
		self.previous_cmd = command
		if self.always_simon:
			self.current_cmd = "simon says, " + self.command_2_speech.get(command)
		else:
			self.current_cmd = random.choice(["simon says, ", ""]) + self.command_2_speech.get(command)
		if "simon says, " in self.current_cmd:
			self.simon_or_naw = True
		else:
			self.simon_or_naw = False
		for key, action in self.command_2_motion.items():
			if command == action[0]:
				self.current_act = key
		self.say_pub.publish(self.current_cmd)
		time.sleep(2)


	def simon_check_response(self):
		"""
		If Edwin is Simon:

		This function checks to see if the players have followed Edwin's cmd
		This relies on skeleton tracker to be functional

		To be implemented when Edwin is Simon
		"""
		pass


	def player_listen_for_simon(self):
		"""
		If Edwin is the player:
		This function listens to a spoken Simon command from user

		This relies on stt_engine to be running
		"""
		self.ready_to_listen = True

		#do nothing while waiting for the speech command
		while not self.heard_cmd:
			continue

		if "simon says" in self.heard_cmd:
			self.simon_or_naw = True
			player_speak = self.heard_cmd.replace("simon says ", "")
		else:
			self.simon_or_naw = False
			player_speak = self.heard_cmd

		action = None
		motions = self.command_2_motion.keys()
		for motion in motions:
			motion = motion.split(" ")
			for word in motion:
				if word in player_speak:
					for key in motions:
						if word in key:
							action = self.command_2_motion.get(key, None)
							break

		if action:
			self.current_act = action[0]
			print "GOT CMD: ", self.current_act
		elif self.no_mistakes:
			statement = "I did not catch that, could you repeat your command?"
			self.say_pub.publish(statement)
			time.sleep(2)
			self.missed_statement = True
		else:
			self.current_act = None

		self.ready_to_listen = False
		self.heard_cmd = None



	def player_follow_simon_cmd(self, difficulty):
		"""
		If Edwin is the player:

		This function takes the command from listen_for_simon() and interprets
		The resulting command is sent to arm_node for physical motion
		"""
		if self.simon_or_naw and self.current_act:
			self.arm_act = self.command_dictionary.get(self.current_act, None)
			if self.arm_act is None:
				return
			else:
				self.arm_act = self.arm_act
			print "GOT MOTION: ", self.arm_act
			self.behavior_pub.publish(self.arm_act)
			self.check_completion()
			time.sleep(1)
			self.current_act = None

		elif self.current_act:
			chance = np.random.rand()
			if chance >= difficulty:
				self.arm_act = self.command_dictionary.get(self.current_act, None)
				if self.arm_act is None:
					return
				else:
					self.arm_act = self.arm_act
				print "GOT MOTION: ", self.arm_act
				self.behavior_pub.publish(self.arm_act)
				self.check_completion()
				time.sleep(1)
				self.current_act = None

			else:
				self.current_act = None
				statement = random.choice(["Haha, you can't fool me", "Gotcha ya, enough playing around", "Get wrecked, I know what you're doing", "Can't fool me"])
				self.say_pub.publish(statement)
				time.sleep(2)


	def run(self):
		"""
		Generic method, to be implemented by the children classes
		"""
		pass



class AutonomousSimon(SimonSays):
	"""
	Game.py Mode when Edwin plays against himself, meaning:
 	1. He will say a Simon command
	2. He will then move based on his own command
	"""
	def __init__(self):
		super(AutonomousSimon, self).__init__()


	def run(self):
		time.sleep(2)
		print "playing by myself"

		move = "get_set"
		self.behavior_pub.publish(move)
		self.check_completion()
		time.sleep(2)
		statement = random.choice(["I am going to demo Simon Says.", "Watch me play Simon Says", "Simon Says is cool! Watch this"])
		self.say_pub.publish(statement)
		time.sleep(2)

		while self.max_turns > 0:
			self.max_turns -= 1

			self.simon_say_command()
			time.sleep(1)
			self.player_follow_simon_cmd(1)

		move = "done_game"
		self.behavior_pub.publish(move)
		self.check_completion()
		statement = random.choice(["Finished a game. Hope you enjoyed!", "Whew, that was easy", "Fun fun fun"])
		self.say_pub.publish(statement)
		time.sleep(3)

		statement = "look"
		self.behavior_pub.publish(statement)
		self.check_completion()
		print "Finished playing by myself, hope you enjoyed the demo!"



class EdwinSimon(SimonSays):
	"""
	Game.py mode where Edwin is Simon, meaning:
	1. Edwin will say a Simon command
	2. Edwin will check if the user has performed the command
	3. Game.py continues for 5 turns or until user fails to perform command
	"""
	def __init__(self, rospy):
		super(EdwinSimon, self).__init__(rospy=rospy)

		# additional variables specific to when Edwin is Simon

		# communicator to finding gestures
		self.ctr_pub = rospy.Publisher('/all_control',String, queue_size=10)

		# finds gestures
		rospy.Subscriber("/skeleton_detect", String, self.gest_callback, queue_size = 10)

		self.first = True           # makes sure the first go is always simon says
		self.gesture = ""			# stores the found gesture
		self.msg = ""				# message variable for self.ctr_pub
		self.simonless_gest = None	# stores the previous simon says for finding stillness


	def gest_callback(self,data):
		"""
		receives the data from what gesture was found from the user, who is the player
		"""

		self.gesture = data.data
		print self.gesture


	def simon_say_command(self):
		"""
		If Simon is Edwin:
		This function issues a Simon command.
		"""

		command = random.choice(self.command_2_speech.keys())

		# makes sure that command is not the same twice in a row
		while command == self.previous_cmd:
			command = random.choice(self.command_2_speech.keys())
		self.previous_cmd = command

		#makes sure that first command contains 'simon says'
		if self.first == True:
			self.current_cmd = "simon says, " + self.command_2_speech.get(command)
			self.first = False
		else:
				self.current_cmd = random.choice(["simon says, ", ""]) + self.command_2_speech.get(command)

		self.say_pub.publish(self.current_cmd)
		time.sleep(4)
		if any(word in self.current_cmd for word in ["wave", "disco", "bow", "hug"]):
			#publishes to all_control that tells it to go and wait 4 seconds for complicated gestures
			self.msg = "gesture_detect:go 4"
		else:
			self.msg = "gesture_detect:go 4"

		self.ctr_pub.publish(self.msg)



	def simon_check_response(self):
		"""
		If Simon is Edwin:
		This function checks to see if the players have followed Edwin's cmd
		This relies on skeleton tracker to be functional
		"""
		self.looking = True
		if "simon says" in self.current_cmd:
			command_gest = self.current_cmd.replace("simon says, ","")
			for key,value in self.command_2_speech.items():
				if value == command_gest:
					for gesture, speech in self.command_dictionary.items():
						if speech == key:
							command_gest = gesture

			#compares command to the gesture received from subscriber
			print "COMMAND", command_gest
			print "GESTURE", self.gesture
			if command_gest  == self.gesture:
				print('Good job! Simon')
				self.simonless_gest = self.gesture
				self.no_mistakes = True
			else:
				print('Try again! Simon')
				self.no_mistakes = False
		else:
			print "SIMONLESS", self.simonless_gest
			print "GESTURE", self.gesture
			if self.simonless_gest == self.gesture:
				print('Good job! Simonless')
				self.no_mistakes = True
			else:
				print('Try again! Simonless')
				self.no_mistakes = False

		self.looking = False


	def run(self):
		"""
		main run loop
		"""

		time.sleep(2)
		print "playing as Simon"

		move = "leader"
		self.behavior_pub.publish(move)
		self.check_completion()
		time.sleep(1)
		statement = random.choice(["Alright, I will be Simon. Ready? Set? Let's play!", "Okay I'm Simon. Are you ready? Let's go!", "Who's Simon? I'm Simon. Hold on to your socks. Here we go!"])
		self.say_pub.publish(statement)
		time.sleep(6)

		self.ctr_pub.publish("gesture_detect:go 1")

		while self.max_turns > 0 and self.no_mistakes:
			self.max_turns -= 1

			self.simon_say_command()
			if any(word in self.current_cmd for word in ["wave", "disco", "bow", "hug"]):
				time.sleep(4)
			else:
				time.sleep(4)


			self.simon_check_response()

		if not self.no_mistakes:
			statement = random.choice(["Nice try, but you messed up. Game.py over.", "Get wrecked, baby, it's over.", "Hahahahah, I'm too quick for you! Sorry, but you lost"])
			move = "gloat"
		else:
			statement = random.choice(["Congratulations, you finished the game!", "Nice job, you completed my game!", "Wow so good, you're done without a sweat!"])
			move = "praise"

		self.ctr_pub.publish("gesture_detect:stop")

		self.behavior_pub.publish(move)
		self.check_completion()
		self.say_pub.publish(statement)
		time.sleep(2)

		statement = "look"
		self.behavior_pub.publish(statement)
		self.check_completion()

		print "Finished playing with player, hope you enjoyed!"



class EdwinPlayer(SimonSays):
	"""
	Game.py mode where Edwin is the player, meaning:
	1. Edwin will listen for a Simon command from the user
	2. Edwin will perform the command based on what he heard
	3. Game.py continues until the user says Edwin is wrong, or until 5 turns
	"""
	def __init__(self, difficulty):
		super(EdwinPlayer, self).__init__()
		self.difficulty = float(difficulty) * 0.1


	def run(self):
		time.sleep(2)
		print "playing as Player"

		move = "get_set"
		self.behavior_pub.publish(move)
		self.check_completion()
		time.sleep(1)
		statement = random.choice(["Alright, I will be the player. Ready? Set? Let's play!", "I'm the player. Gimme your best shot! Let's go!", "Okay okay I'll be the player. I'm ready when you are."])
		self.say_pub.publish(statement)
		time.sleep(2)

		while self.max_turns > 0 and self.no_mistakes:
			self.max_turns -= 1

			self.player_listen_for_simon()
			time.sleep(2)
			self.player_follow_simon_cmd(self.difficulty)

			if self.missed_statement == True:
				self.max_turns += 1
				self.missed_statement = False
			else:
				print "next turn please"

		if not self.no_mistakes:
			statement = random.choice(["Oh no, I messed up. Good game though.", "Darn, I'm bad. Nice game.", "Ugh, a mistake! Excuse me while I go cry in a corner..."])
			move = "sad"
		else:
			statement = random.choice(["Yay! We made it to the end of the game. It was fun!", "Oh yes! I finished woo!", "Ohohohohoho baby I'm so good! I did it! Good game."])
			move = "praise"

		self.behavior_pub.publish(move)
		self.check_completion()
		self.say_pub.publish(statement)
		time.sleep(2)

		statement = "look"
		self.behavior_pub.publish(statement)
		self.check_completion()
		print "Finished playing with Simon, hope you enjoyed!"



if __name__ == '__main__':
	mode = raw_input("Is Edwin simon or the player?\n")

	if mode == PLAYER_NAME:
		difficulty = raw_input("How good should Edwin be?\n")
		game = EdwinPlayer(difficulty)
	elif mode == SIMON_NAME:
		game = EdwinSimon()
	else:
		game = AutonomousSimon()
		game.always_simon = True

	game.run()
