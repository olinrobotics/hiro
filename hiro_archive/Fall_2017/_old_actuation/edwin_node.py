#!/usr/bin/env python
from __future__ import absolute_import
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String, Int16, Header
from sensor_msgs.msg import JointState
import time
from irl.srv import arm_cmd

class ArmCommands(object):
    def __init__(self):
        rospy.init_node('robot_arm', anonymous=True)
        s = rospy.Service('arm_cmd', arm_cmd, self.arm_callback)
        print "Service ONLINE"

        rospy.Subscriber('/arm_cmd', String, self.arm_callback, queue_size=1)
        self.debug_pub = rospy.Publisher('arm_debug', String, queue_size=10)
        self.status_pub = rospy.Publisher('/arm_status', String, queue_size=10)

        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.joint_state_msg = JointState()

        self.debug = False
        self.plan = []
        self.arm = st.StArm()
        print "CALIBRATING"
        self.arm.initial_calibration()
        self.arm.start()

        print "ARM SPD IS: ", self.arm.get_speed()
        print "ARM ACCEL IS: ", self.arm.get_accel()

        self.arm.set_speed(10000)
        self.arm.home()
        print "HOMING"

        self.debug_pub.publish("HOMING DONE")

    def arm_callback(self, cmdin):
        self.status_pub.publish("busy")
        self.arm.joint()
        cmd = cmdin.cmd
        cmd = str(cmd).replace("data: ", "")
        if len(cmd.split(':: ')) > 1:
            param = cmd.split(':: ')[1]
            cmd = cmd.split(':: ')[0]
        print cmd
        if cmd == "de_energize":
            self.arm.de_energize()
        elif cmd == "energize":
            self.arm.energize()
        elif cmd == "where":
            location = self.arm.where()

            #print 'location', location

        elif cmd == "create_route":
            print "CREATING NEW ROUTE"
            param = param.split("; ")
            route_name = param[0]

            commands = []
            numbers = param[1].split(", ")

            if len(numbers)%6 != 0:
                print "INVALID ROUTE"
                return

            i = 0
            while i < len(numbers):
                route = []
                j = 0
                while j < 6:
                    route.append(int(numbers[i+j]))
                    j += 1
                commands.append(route)
                i += j

            print "CREATING ROUTE: ", route_name
            print "CMDS: ", commands
            self.arm.create_route(route_name, commands)

        elif cmd == "calibrate":
            self.arm.calibrate()
        elif cmd == "home":
            self.arm.home()
        elif cmd == "get_speed":
            speed = self.arm.get_speed()
        elif cmd == "set_speed":
            #SPD is also in units of 1000
            print "setting speed to ", param
            self.arm.set_speed(float(param))
            print "ARM SPD IS: ", self.arm.get_speed()
        elif cmd == "set_point":
            self.arm.set_point(param)
        elif cmd == "get_accel":
            accel = self.arm.get_accel()
        elif cmd == "set_accel":
            #ACCEL is also in units of 1000
            print "setting accel to ", param
            self.arm.set_accel(float(param))
        elif cmd == "run_route":
            res = self.arm.run_route(param)

            if not res:
                self.debug_pub.publish("ROUTE NOT FOUND: " + param)

        elif cmd == "move_to":
            #NOTE: move_to is in units of mm
            temp = param.split(", ")
            x = temp[0]
            y = temp[1]
            z = temp[2]
            pitch = temp[3]
            self.arm.move_to(x,y,z,self.arm.debug)

        elif cmd == "rotate_wrist":
            self.arm.rotate_wrist(param)

        elif cmd == "rotate_wrist_rel":
            self.arm.rotate_wrist_rel(param)

        elif cmd == "rotate_hand":
            self.arm.rotate_hand(param)

        elif cmd == "rotate_elbow":
            self.arm.rotate_elbow(param)

        elif cmd == "rotate_shoulder":
            self.arm.rotate_shoulder(param)

        elif cmd == "rotate_waist":
            self.arm.rotate_waist(param)

        elif cmd == "rotate_waist_rel":
            print "RELATIVE WA ROTATION"
            self.arm.rotate_waist(param)

        elif cmd == "rotate_hand_rel":
            self.arm.rotate_hand_rel(param)

        elif cmd == "sleeping":
            time.sleep(float(param))
        elif cmd == "get_joint_states":
            joint_states = self.arm.get_joint_states()

            h = Header()
            h.frame_id = "base_link"
            h.stamp = rospy.Time.now()

            self.joint_state_msg.header = h
            self.joint_state_msg.name = ['joint_1','joint_2','joint_3','joint_4','joint_5']
            # TODO : fix these names ... here for compatibility with URDF model
            self.joint_state_msg.position = joint_states
            self.joint_state_msg.velocity = [0 for _ in xrange(5)]
            self.joint_state_msg.effort = [0 for _ in xrange(5)]

            self.joint_state_pub.publish(self.joint_state_msg)

        elif cmd == "test":
            self.arm.test()


        self.status_pub.publish("free")
        return ["I have completed the command"]


    def run(self):
        print "Service is ready to go"
        rospy.spin()
        # r = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     r.sleep()

if __name__ == "__main__":
    arm_eng = ArmCommands()
    arm_eng.run()
