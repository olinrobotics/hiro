#!/usr/bin/python
import roslib; roslib.load_manifest('jimmy')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Char
from jimmy.msg import *
import random

def interaction_callback(data):
    global message
    wave = data.wave
    hello = data.hello
    goodbye = data.goodbye
    #our message fields are all booleans so we can do easy if statements
    if wave:
        print "waved!"
        message = 6#random.randint(8, 11) #sets messages as desired as reponses to detected gestures
    if hello:
        print "hello!"
        message = 13
    if goodbye:
        print "goodbye"
        message = 14
    if not goodbye and not hello and not wave:
        print "none work"
        message = 0
    print message

def publisher():
    global message
    rospy.init_node("interaction_node", anonymous = True)
    rospy.Subscriber("/detected_gestures", jimmy_gesture, interaction_callback) #subscribing to detected gestures from detectfinger
    pub = rospy.Publisher('/jimmy_idle', jimmy_gesture)
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        if message == 6:
            msg = message
            pub.publish(msg)
        r.sleep()

if __name__ == "__main__":
    message = 0
    publisher()