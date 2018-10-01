#!/usr/bin/env python
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String, Int16
import time

class ArmCommands:
    def __init__(self):
        print "starting ROBOFORTH debug"
        # rospy.init_node('robot_arm', anonymous=True)

        self.arm = st.StArm()

        # print "ARM SPD IS: ", self.arm.get_speed()
        # print "ARM ACCEL IS: ", self.arm.get_accel()
        #
        # self.arm.set_speed(10000)

    def arm_callback(self, cmd):
        self.arm.joint()
        print "GOT COMMAND: ", cmd
        if cmd == "de_energize":
            self.arm.de_energize()
        elif cmd == "energize":
            self.arm.energize()
        elif cmd == "where":
            location = self.arm.where()
            print location
        elif cmd == "start":
            print "CALIBRATING"
            self.arm.initial_calibration()
            self.arm.start()
        else:
            self.arm.execute_command(cmd)

    def run(self):
        while True:
            cmd_in = str(raw_input("ROBOFORTH COMMAND: "))
            self.arm_callback(cmd_in)

if __name__ == "__main__":
    arm_eng = ArmCommands()
    arm_eng.run()
