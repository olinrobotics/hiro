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

# python movement.py _robot_ip:=10.42.0.54
# pollux: 10.42.0.175, castor: 10.42.0.54
class ChessPlanner():
   
    def __init__(self):
        rospy.init_node("movement", anonymous=True, disable_signals=False)

        # two ways of controlling the arm with coordinates / joint behaviors
        self.coordinates_pub_castor = rospy.Publisher("/coordinates_cmd_castor", String, queue_size=10)
        self.joints_pub_castor = rospy.Publisher("/behaviors_cmd_castor", String, queue_size=10)

        # controller_status determines when Perception start looking for a new goal
        self.status_pub = rospy.Publisher("/controller_status", Bool, queue_size=10)

        # setting the origin (left top) of the chess board
        x_init = 0.024
        y_init = -0.652
        z_init = 0.306
        global origin
        origin = [x_init, y_init, z_init]

        # setting the units for the chess board
        global unit
        unit = 0.0625

    def wait(self, second):
        for i in range(second):
            print("starting operation in "+str(second-i)+" seconds")
            time.sleep(1)
        
    def move(self, x, y):
        if x>7 or x<0 or y>7 or y<0:
            raise ValueError('Parameter should be over 0 and under 7')
            return
        
        x_coord = origin[0] - y * unit
        y_coord = origin[1] + x * unit
        z_coord = origin[2]

        if x_coord<-0.2 or x_coord>0.4 or y_coord<-0.75 or y_coord>0.2:
            raise ValueError('Wrong Range!')
            return

        msg = str(x_coord) + ' ' + str(y_coord) + ' ' + str(z_coord)
        print('Sending Castor:' + msg)
        self.coordinates_pub_castor.publish(msg)
        time.sleep(5)
    
    def grip(self):
        old_pose = self.get_position()
        print(old_pose)
        x_coord = old_pose[0]
        y_coord = old_pose[1]
        z_coord = old_pose[2]

        if z_coord > 0.4:
            raise ValueError('The arm is too far up')
            return
        
        z_coord2 = 0.220

        msg = str(x_coord) + ' ' + str(y_coord) + ' ' + str(z_coord2)
        print('Sending Castor:' + msg)
        self.coordinates_pub_castor.publish(msg)
        time.sleep(3)

        msg = str(x_coord) + ' ' + str(y_coord) + ' ' + str(z_coord)
        print('Sending Castor:' + msg)
        self.coordinates_pub_castor.publish(msg)
        time.sleep(5)

    def move_ordinary(self, position):
        if len(position) <> 4:
            raise ValueError('Position should be in form of "a2a4", indicating the begining position and the end position')
        
        change_to_cord = {'a':0, 'b':1, 'c':2, 'd':3, 'e':4, 'f':5, 'g':6, 'h':7}

        cord_position = [change_to_cord[position[0]], int(position[1])-1, change_to_cord[position[2]], int(position[3])-1]

        self.move(cord_position[0], cord_position[1])
        self.grip()
        self.move(cord_position[2], cord_position[3])
        self.grip()
        
    def get_position(self):
        return self.coordinator.getl()

    def run(self):
        print("Chess Planner running")
        tcp_ip = "10.42.0.54"
        self.coordinator = urx.Robot(tcp_ip)
        switch = 3
        stop_switch = 0
        while stop_switch == 0 and not rospy.is_shutdown():
            try:
                if switch == -1:
                    print("moving the arm to hovering position in 5 second")
                    time.sleep(5)
                    self.joints_pub_castor.publish("chess_hover")
                    time.sleep(10)
                    switch += 1

                if switch == 0:
                    print("moving the arm to the origin in 5 second")

                    x_coord = origin[0]
                    y_coord = origin[1]
                    z_coord = origin[2]

                    msg = str(x_coord) + ' ' + str(y_coord) + ' ' + str(z_coord)
                    print('Sending Castor:' + msg)
                    
                    time.sleep(5)
                    self.coordinates_pub_castor.publish(msg)
                    time.sleep(10)
                    switch += 3
                
                if switch == 1:
                    print("moving chess piece to coordinate 4,4")

                    self.wait(5)
                    self.move(1,1)
                    switch += 1

                if switch == 2:
                    print("Gripping in 5 second")
                    time.sleep(5)
                    self.grip()
                    stop_switch += 1
                
                if switch == 3:
                    self.wait(5)
                    self.move_ordinary('c3a4')
                    self.wait(10)
                
                if switch == 4:
                    self.ur5_arm_node.g
                    
            except KeyboardInterrupt:
                self.coordinator.close()
                print("interrupting")
                break
        print("the program ended!")
        self.coordinator.close()

if __name__ == "__main__":
    cp = ChessPlanner()
    cp.run()
