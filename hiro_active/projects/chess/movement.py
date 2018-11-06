#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import time
import sys
import math
from std_msgs.msg import String, Bool, Int32
rospack = rospkg.RosPack()
PACKAGE_PATH = rospack.get_path("hiro")
sys.path.append(PACKAGE_PATH + '/projects')
from ur5_arm_node import Arm
import urx

# python path_planner.py _robot_ip:=10.42.0.54
# pollux: 10.42.0.175, castor: 10.42.0.54
class ChessPlanner():
   
    def __init__(self):
        rospy.init_node("movement", anonymous=True)

        # two ways of controlling the arm with coordinates / joint behaviors
        self.coordinates_pub_castor = rospy.Publisher("/coordinates_cmd_castor", String, queue_size=10)
        self.joints_pub_castor = rospy.Publisher("/behaviors_cmd_castor", String, queue_size=10)

        # controller_status determines when Perception start looking for a new goal
        self.status_pub = rospy.Publisher("/controller_status", Bool, queue_size=10)

    def run(self):
        print("Chess Planner running")
        tcp_ip = "10.42.0.54"
        self.coordinator = urx.Robot(tcp_ip)
        while not rospy.is_shutdown():
            try:
                self.joints_pub_castor.publish("chess_hover")
                time.sleep(5)
                old_pose = self.coordinator.getl()
                print(old_pose)
                self.joints_pub_castor.publish("chess_hover2")
                time.sleep(5)
                new_pose = self.coordinator.getl()
                print(new_pose)
               

               #msg = str(real_coord.x) + ' ' + str(real_coord.y) + ' ' + str(self.curr_location[2])
               #print('Sending Pollux:' + msg)
               #self.coordinates_pub_pollux.publish(msg)
               #self.check_pollux()

               #real_coord.x -= self.realXP_offset
               #real_coord.y -= self.realYP_offset

            except KeyboardInterrupt:
                print("interrupting")
                break

if __name__ == "__main__":
    cp = ChessPlanner()
    cp.run()
