#!/usr/bin/env python
import rospy
import random
from time import sleep
from std_msgs.msg import String
from espeak import espeak

class SpeechEngine:
    def __init__(self):
        rospy.init_node("tts_engine", anonymous=True)
        rospy.Subscriber("edwin_speech_cmd", String, self.callback, queue_size=10)

        espeak.set_parameter(espeak.Parameter.Rate,150)
        espeak.set_parameter(espeak.Parameter.Pitch,50)

        espeak.set_voice("en")
        print "Ready to speak!"

    def say(self, text_to_say):
        espeak.synth(text_to_say)

    def callback(self, data):
        self.say(data.data)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    speech_engine = SpeechEngine()
    speech_engine.run()
