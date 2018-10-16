#!/usr/bin/env python

import roslib; roslib.load_manifest('jimmy')
import rospy
import random
from time import sleep
from std_msgs.msg import String
from espeak import espeak
from jimmy.msg import *

def say(something): #I'll give it up for you.
    espeak.synth(something)

def remote_say(data):
    print "in remote_say"
    raw = str(data).lower()
    sentance = raw.split(' ',1)[1]
    print sentance
    say(sentance)

def callback(data):
#    while not rospy.is_shutdown():
    raw = str(data).lower()
    sentance = raw.split(' ',1)[1]
    print sentance
#    say(sentance)
    if 'hello' in sentance:
        movegesture(16)
        sleep(2)
        say("Hello! How are you.")
        return
    if 'thank' in sentance:
        movegesture(18)
        sleep(2)
        say("You're welcome.")
        return
    if 'your name' in sentance:
        movegesture(random.randint(2,6))
        sleep(1)
        say("My name is Jimmy.")
        return
    if "haha" in sentance:
        movegesture(18)
        sleep(2)
        say("haha, i am laughing")
        return
    else:
        say("I'm sorry.")
        movegesture(19)
        sleep(1)
        say("I do not understand.")
        return
#    if 'move' in sentance:
#        say("I can do that.")
#        move("RightElbow", 0.5)
#        print "Moving!"
#    if 'back' in sentance:
#        say("I can do that.")
#        move("RightElbow", 0)

def move(servo, position):
    pub = rospy.Publisher("jimmy_move_servo", jimmy_servo)
    r = rospy.Rate(10) # 10hz
    msg = jimmy_servo()
    msg.servo_names.append(servo)
    msg.positions.append(position)
    pub.publish(msg)
#    rospy.spin()
    print "Message published!"

def movegesture(gesture):
    pub = rospy.Publisher("jimmy_send_gesture", jimmy_gesture)
    r = rospy.Rate(10)
    msg = jimmy_gesture()
    msg.cmd = gesture
    pub.publish(msg)
    print "Gesture published"

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("conversation", String, callback)
    rospy.Subscriber("remote_speech", String, remote_say)
    rospy.spin()

if __name__ == '__main__':
    try:
        espeak.set_parameter(espeak.Parameter.Rate,150)
        espeak.set_parameter(espeak.Parameter.Pitch,99)
#        espeak.set_parameter(espeak.Parameter.Wordgap,)
        espeak.set_voice("en")
        print "Ready to speak!"
        listener()
    except rospy.ROSInterruptException: pass
