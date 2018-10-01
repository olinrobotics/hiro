#!/usr/bin/env python

"""
Adapted code from Olin College OccamLab
Github repo found here: https://github.com/occamLab/eye-helper-cv
"""
import roslib; roslib.load_manifest('jimmy')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Char
import random

import subprocess
import time

class audio_object:
    def __init__(self, name):
        self.name = name
        self.path = "../media" # TODO: fix this so it doesn't use '..'
        self.filename = "{}/{}.mp3".format(self.path, self.name)
        self.player = 'mplayer'
        self.playme = False


    def play_wave(self):
        """
        plays an inputted wav file
        """
        print self.filename
        cmd = '{} {}'.format(self.player, self.filename)
        #popen = subprocess.Popen([self.player, self.filename, "-ss", "30"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()
        return
        #popen.stdin.write("q")

    def update(self):
        if self.playme:
            time.sleep(1)
            print "playing sound file ", self.filename
            self.play_wave()
            self.playme = False
            return

def sound_callback(data):
    print "in sound_callback"
    global all_sounds
    global all_sounds_list
    print data
    char = data.data
    print char
    if char == 8:
        print "play huh"
        all_sounds["huh"].playme = True
    elif char == 9:
        print "play tada"
        all_sounds["tada"].playme = True
    elif char == 10:
        print "play yoohoo"
        all_sounds["youhoo"].playme = True
    elif char == 11:
        print "play ooh"
        all_sounds["ooh"].playme = True
    elif char == 13:
        print "play whistle"
        all_sounds["whistle"].playme = True

    for sound in all_sounds_list:
        print "updating ", sound.filename
        sound.update()
#    return

def subscriber():
    rospy.init_node("sound_player", anonymous = True)
    rospy.Subscriber("/sound_node", Char, sound_callback) #subscribing to detected gestures from detectfinger
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
       r.sleep()
       rospy.spin()


if __name__ == '__main__':
    tada = audio_object("tada")
    huh = audio_object("huh")
    whistle = audio_object("whistle")
    yoohoo = audio_object("youhoo")
    ooh = audio_object("ooh")

    all_sounds = {"tada":tada, "huh":huh, "whistle":whistle, "youhoo":yoohoo, "ooh":ooh}
    all_sounds_list = [tada, huh, whistle, yoohoo, ooh]

    # huh.playme = True

    # huh.update()
    while not rospy.is_shutdown():
        subscriber()