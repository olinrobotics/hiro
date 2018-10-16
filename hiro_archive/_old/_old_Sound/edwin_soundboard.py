#!/usr/bin/env python
import rospy
import rospkg
import os
from std_msgs.msg import String
from edwin.msg import *
import time
import subprocess

class AudioObject:
	def __init__(self, path, name):
		self.filename = "{}/{}".format(path, name)
		self.player = 'mplayer'

	def play(self):
		cmd = '{} {}'.format(self.player, self.filename)
		#popen = subprocess.Popen([self.player, self.filename, "-ss", "30"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

		popen = subprocess.Popen(cmd, shell=True)
		popen.communicate()
		return 1
		#popenj.stdin.write("q")

class SoundBoard:
	def __init__(self):
		rospy.init_node('edwin_soundboard', anonymous = True)
		rospy.Subscriber('edwin_sound', String, self.sound_callback)
		self.sound_library = {}
		self.create_sound_library()

	def create_sound_library(self): #Reads all the files in media, instantiates them as audio_objects
		rospack = rospkg.RosPack()
		PACKAGE_PATH = rospack.get_path("edwin")
		MEDIADIR = os.path.join(PACKAGE_PATH, "scripts/sound/media")

		files = os.listdir(MEDIADIR)
		for f in files:
			self.sound_library[f] = AudioObject(MEDIADIR, f)

	def sound_callback(self, data):
		sound_play = data.data + ".wav"
		print "PLAYING: ", sound_play
		self.sound_library[sound_play].play()

if __name__ == '__main__':
	sound = SoundBoard()
	rospy.spin()
