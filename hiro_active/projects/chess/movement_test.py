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
import chess
import chess.uci

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
        x_init = 0.209
        y_init = -0.727
        z_init = 0.300
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

        msg = str(x_coord) + ' ' + str(y_coord) + ' ' + str(z_coord)
        print('Sending Pollux:' + msg)
        self.coordinates_pub_castor.publish(msg)
        time.sleep(5)

    def move_outside(self, x, y):
        
        x_coord = origin[0] - y * unit
        y_coord = origin[1] + x * unit
        z_coord = origin[2]

        msg = str(x_coord) + ' ' + str(y_coord) + ' ' + str(z_coord)
        print('Sending Pollux:' + msg)
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
        print('Sending Pollux:' + msg)
        self.coordinates_pub_castor.publish(msg)
        time.sleep(3)

        msg = str(x_coord) + ' ' + str(y_coord) + ' ' + str(z_coord)
        print('Sending pollux:' + msg)
        self.coordinates_pub_castor.publish(msg)
        time.sleep(5)
    
    def release(self):
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
        print('Sending Pollux:' + msg)
        self.coordinates_pub_castor.publish(msg)
        time.sleep(3)

        msg = str(x_coord) + ' ' + str(y_coord) + ' ' + str(z_coord)
        print('Sending pollux:' + msg)
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
        self.release()
    
    def move_capture(self, position, side, num):
        if len(position) <> 4:
            raise ValueError('Position should be in form of "a2a4", indicating the begining position and the end position')
        
        change_to_cord = {'a':0, 'b':1, 'c':2, 'd':3, 'e':4, 'f':5, 'g':6, 'h':7}

        cord_position = [change_to_cord[position[0]], int(position[1])-1, change_to_cord[position[2]], int(position[3])-1]

        out_position = [0,0]

        if side == "b":
            out_position[0] = 0
            out_position[1] = 9
        else:
            out_position[0] = 0
            out_position[1] = -2

        if num >= 8:
            if side == "b":
                out_position[1] +=1
            else:
                out_position[1] -=1
            num = num - 8
        
        out_position[0] += num
        
        self.move(cord_position[2], cord_position[3])
        self.grip()
        self.move_outside(out_position[0], out_position[1])
        self.release()

        self.move(cord_position[0], cord_position[1])
        self.grip()
        self.move(cord_position[2], cord_position[3])
        self.release()

    def to_hovering(self):
        print("moving the arm to hovering position")
        self.joints_pub_castor.publish("chess_hover3")
        time.sleep(5)

    def to_origin(self):
        print("moving the arm to the origin in 5 second")

        x_coord = origin[0]
        y_coord = origin[1]
        z_coord = origin[2]

        msg = str(x_coord) + ' ' + str(y_coord) + ' ' + str(z_coord)
        print('Sending Pollux:' + msg)
                    
        time.sleep(5)
        self.coordinates_pub_castor.publish(msg)
        time.sleep(5)

    def get_position(self):
        return self.coordinator.getl()

    def run(self):
        print("Chess Movement running")
        tcp_ip = "10.42.0.54"
        self.coordinator = urx.Robot(tcp_ip)

        board = chess.Board()

        engine = chess.uci.popen_engine("stockfish")
        engine.uci()
        engine.position(board)

        side = "w"
        num_b = 0
        num_w = 0

        switch = 0

        while not board.is_game_over() and not rospy.is_shutdown():
            try:
                if switch == 0:

                    self.to_hovering()
                    print("Next Move in 5 second")
                    engine.position(board)
                    move = engine.go(movetime=1000)

                    capture_status = board.is_capture(move.bestmove)
                    board.push(move.bestmove)
                    print(board)
                    time.sleep(5)

                    if capture_status == True and side == "w":
                        self.move_capture(str(move.bestmove), side, num_w)
                        num_w += 1
                    elif capture_status == True and side == "b":
                        self.move_capture(str(move.bestmove), side, num_b)
                        num_b += 1
                    else:
                        self.move_ordinary(str(move.bestmove))

                    if side == "b":
                        side = "w"
                    else:
                        side = "b"
                elif switch == 1:
                    #self.wait(5)
                    #self.to_hovering()
                    self.move_capture("a1b4", "w", 0)

        
            except KeyboardInterrupt:
                self.coordinator.close()
                print("interrupting")
                break
        print("the program ended!")
        self.coordinator.close()

if __name__ == "__main__":
    cp = ChessPlanner()
    cp.run()
