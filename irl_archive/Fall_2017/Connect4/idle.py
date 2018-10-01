#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import time
import numpy as np

"""
Kevin Zhang

The Idle Module, a script that can be toggled on and off and allows for Draco to
just move around lazily when not engaged with a user but still on and waiting for something
to happen.

This script is meant to be imported as a package to the brain, but it can be tested
in isolation, just change the self.state to True

How it works:
1. sits around and does nothing until told to by a publisher in brain (or elsewhere)
2. when state if "go", begins idling, or drawing from 10 different idle movements and
excuting them randomly.
3. continues until told not to, probably when an activity is going to begin, like some
interaction with a user
"""

class Idler(object):
    """
    the Idler class, which allows Draco to be "idle", which consists of 10 lazy/chill
    movements to be done when he's not actively actively doing anything
    """

    def __init__(self):
        rospy.init_node("Idler")

        self.ur5_commander = rospy.Publisher("behaviors_cmd", String, queue_size=10)

        rospy.Subscriber("idle_state", String, self.state_callback)
        rospy.Subscriber("arm_status", String, self.status_callback)

        self.status = True
        self.state = False
        self.idle_commands = ["idle_stare_1", "idle_stare_2", "idle_stare_3", "idle_curiosity", "idle_sniff",
                            "idle_wander", "idle_sneeze", "idle_butt_wiggle", "idle_head_bobble", "idle_sleep"]
        self.idle_command = None
        self.previous_command = None


    def status_callback(self, data):
        """
        callback for the status of the arm
        """

        arm = data.data
        if arm == "free":
            self.status = True
        else:
            self.status = False


    def state_callback(self, data):
        """
        callback for the state of this script
        """

        command = data.data
        if command == "go":
            self.state = True
        else:
            self.state = False


    def check_for_completion(self):
        """
        checks to make sure arm is done with previously sent command before continuing
        """

        time.sleep(3)
        while not self.status:
            pass


    def idle(self):
        """
        randomly choose from the 10 idle motions, and then performs it.
        randomly waits for a bit before going back to for loop
        """

        while self.idle_command == self.previous_command:
            self.idle_command = np.random.choice(self.idle_commands)

        print "Running ", self.idle_command
        self.ur5_commander.publish(self.idle_command)
        self.check_for_completion()

        self.previous_command = self.idle_command

        pause = np.random.choice(range(3,6))
        time.sleep(pause)


    def run(self):
        """
        main run function for idle, just sits passively until told to go, at which
        it runs idle()
        """

        print "Idle Module is running"
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                r.sleep()
                if self.state:
                    self.idle()

            except KeyboardInterrupt:
                print "\n Idle module turned off"
                break


if __name__=="__main__":
    i = Idler()
    i.run()
