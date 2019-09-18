#!/usr/bin/env python
"""
By SeungU Lyu & Audrey Lee December 18, 2018

This is the master script of chess play of a robot arm against a human player. 
This script will:
- have own data chessboard used inside the program.
- use openCV to recognize the physical position of chess pieces after a human move.
- compare the position of chess pieces on data chessboard and physical chessboard and recognize which piece moved to which position.
- not require need of recognizing exact types and positions of each chess pieces.
- use stockfish chess engine 10 and python-chess script to calculate the next best move for the robot arm.
- use the calculated next move to physically move the chess pieces by the robot arm.
- end the program when the chess game is over.

Dependencies:
- realsense2_camera: https://github.com/intel-ros/realsense
- librealsense: https://github.com/IntelRealSense/librealsense
- rgbd_launch: https://github.com/ros-drivers/rgbd_launch.git
- starfish UCI chess engine: https://github.com/official-stockfish/Stockfish
- python-chess: https://github.com/niklasf/python-chess
- robotiq 2F-140 Gripper: https://github.com/ros-industrial/robotiq

To use:
- Open Terminal and run the code below:

roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=10.42.0.175
- connect robot with ubuntu by tcp/ip

rosrun hiro ur5_arm_node.py _robot_ip:=10.42.0.175
- run ur5_arm_node.py

rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
- connect 2F-140 gripper with the computer

optional: rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py
- use this to check if the gripper isn't working as intended

python movement.py
- start the program

"""
import OpenCvRealSenseCameras
import chess_calculate

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
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
import cv2

# python movement.py _robot_ip:=10.42.0.54
# pollux: 10.42.0.175, castor: 10.42.0.54
class ChessPlanner():
   
    def __init__(self):
        rospy.init_node("movement", anonymous=True, disable_signals=False)

        # two ways of controlling the arm with coordinates / joint behaviors
        self.coordinates_pub_pollux = rospy.Publisher("/coordinates_cmd_pollux", String, queue_size=10)
        self.joints_pub_pollux = rospy.Publisher("/behaviors_cmd_pollux", String, queue_size=10)

        # controller_status determines when Perception start looking for a new goal
        self.status_pub = rospy.Publisher("/controller_status", Bool, queue_size=10)

        # setting the origin (left top) of the chess board
        x_init = 0.196
        y_init = -0.694
        z_init = 0.350
        global origin
        origin = [x_init, y_init, z_init]

        # setting the units for the chess board, according to the right bottom 
        global x_unit
        x_unit = (0.196+0.234)/7
        global y_unit
        y_unit = (0.694-0.247)/7

        #Gripper Code
        #rospy.init_node('robotiq_gripper_controller')
        self.pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
        self.command = outputMsg.Robotiq2FGripper_robot_output();

        # activate the gripper
        self.command = outputMsg.Robotiq2FGripper_robot_output();
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSP  = 255
        self.command.rFR  = 150
        self.pub.publish(self.command)
        time.sleep(1)

    def wait(self, second):
        # waiting code with countdown. For test purpose
        for i in range(second):
            print("starting operation in "+str(second-i)+" seconds")
            time.sleep(1)
        
    def move(self, x, y):
        # move the robot arm to x,y coordinates. Since the chess FEN starts count from y-axis, the order is opposite from normal xy coordinates. 
        if x>7 or x<0 or y>7 or y<0:
            raise ValueError('Parameter should be over 0 and under 7')
            return
        
        x_coord = origin[0] - y * x_unit
        y_coord = origin[1] + x * y_unit
        z_coord = origin[2]

        msg = str(x_coord) + ' ' + str(y_coord) + ' ' + str(z_coord)
        self.coordinates_pub_pollux.publish(msg)
        time.sleep(4)

    def move_outside(self, x, y):
        # move code without the limitation. Used to remove chess pieces. 
        x_coord = origin[0] - y * x_unit
        y_coord = origin[1] + x * y_unit
        z_coord = origin[2]

        msg = str(x_coord) + ' ' + str(y_coord) + ' ' + str(z_coord)
        self.coordinates_pub_pollux.publish(msg)
        time.sleep(4)
    
    def close_grip(self):
        # close gripper / change its position to 255
        self.command.rPR = 255
        self.pub.publish(self.command)
        time.sleep(2)

    def open_grip(self):
        # open gripper / change its position to 140
        self.command.rPR = 140
        self.pub.publish(self.command)
        time.sleep(2)
    
    def grip(self):
        # grips a chess piece from the current position.
        old_pose = self.get_position()
        x_coord = old_pose[0]
        y_coord = old_pose[1]
        z_coord = old_pose[2]
        
        z_coord2 = 0.275

        msg = str(x_coord) + ' ' + str(y_coord) + ' ' + str(z_coord2)
        self.coordinates_pub_pollux.publish(msg)
        time.sleep(2)
        self.close_grip()

        msg = str(x_coord) + ' ' + str(y_coord) + ' ' + str(z_coord)
        #print('Sending pollux:' + msg)
        self.coordinates_pub_pollux.publish(msg)
        time.sleep(2)
    
    def release(self):
        # releases a chess piece on the current position
        old_pose = self.get_position()
        x_coord = old_pose[0]
        y_coord = old_pose[1]
        z_coord = old_pose[2]
        
        z_coord2 = 0.277

        msg = str(x_coord) + ' ' + str(y_coord) + ' ' + str(z_coord2)
        self.coordinates_pub_pollux.publish(msg)
        time.sleep(2)
        self.open_grip()

        msg = str(x_coord) + ' ' + str(y_coord) + ' ' + str(z_coord)
        self.coordinates_pub_pollux.publish(msg)
        time.sleep(2)

    def move_ordinary(self, position):
        # move the chess code using "a1a4" UCI command, to an empty spot
        if len(position) <> 4:
            raise ValueError('Position should be in form of "a2a4", indicating the begining position and the end position')
        
        change_to_cord = {'a':0, 'b':1, 'c':2, 'd':3, 'e':4, 'f':5, 'g':6, 'h':7}

        cord_position = [change_to_cord[position[0]], int(position[1])-1, change_to_cord[position[2]], int(position[3])-1]

        self.move(cord_position[0], cord_position[1])
        time.sleep(0.5)
        self.grip()
        self.move(cord_position[2], cord_position[3])
        time.sleep(0.5)
        self.release()
    
    def move_capture(self, position, side, num):
        # run if next move is a capture move. removes the chess piece at the destination and move the piece.
        if len(position) <> 4:
            raise ValueError('Position should be in form of "a2a4", indicating the begining position and the end position')
        
        change_to_cord = {'a':0, 'b':1, 'c':2, 'd':3, 'e':4, 'f':5, 'g':6, 'h':7}

        cord_position = [change_to_cord[position[0]], int(position[1])-1, change_to_cord[position[2]], int(position[3])-1]

        out_position = [0,0]

        if side == "b":
            out_position[0] = 0
            out_position[1] = -2
        else:
            out_position[0] = 0
            out_position[1] = 9

        if num >= 8:
            if side == "b":
                out_position[1] +=1
            else:
                out_position[1] -=1
            num = num - 8
        
        out_position[0] += num
        
        self.move(cord_position[2], cord_position[3])
        time.sleep(0.5)
        self.grip()
        self.move_outside(out_position[0], out_position[1])
        time.sleep(0.5)
        self.release()

        self.move(cord_position[0], cord_position[1])
        time.sleep(0.5)
        self.grip()
        self.move(cord_position[2], cord_position[3])
        time.sleep(0.5)
        self.release()

    def to_hovering(self):
        # moves arm to the hovering position
        print("moving the arm to hovering position")
        self.joints_pub_pollux.publish("chess_hover3")
        time.sleep(5)

    def to_origin(self):
        # moves arm to the origin
        print("moving the arm to the origin in 5 second")

        x_coord = origin[0]
        y_coord = origin[1]
        z_coord = origin[2]

        msg = str(x_coord) + ' ' + str(y_coord) + ' ' + str(z_coord)
                    
        time.sleep(5)
        self.coordinates_pub_pollux.publish(msg)
        time.sleep(5)

    def get_position(self):
        #get current x, y, z coordinate of the robot arm
        return self.coordinator.getl()
    
    def enter_mode(self):
        # mode select part for running program. 
        switch = 0
        while True:
            print("Select the mode")
            print("0: direct mode (no reset, not recommended")
            print("1: player turn mode, do not use")
            print("2: reset position and start playing")
            print("3: look at current position and decide")

            number = raw_input("->")

            if int(number) >=0 and int(number) <= 3:
                switch = int(number)
                break
            else:
                print("wrong value")
        
        return switch

    def run(self):
        # main part
        print("Chess Movement running")
        #connect to pollux
        tcp_ip = "10.42.0.175"
        self.coordinator = urx.Robot(tcp_ip)

        #create initial chessboard with initial piece positions
        board = chess.Board()

        #connect stockfish engine with chess-python. Turns on engine so that program knows the engine is running
        engine = chess.uci.popen_engine("stockfish")
        engine.uci()
        #set initial engine position
        engine.position(board)

        #initialize number of dead pieces for each side
        num_b = 0
        num_w = 0

        #select mode before while loop starts
        switch = self.enter_mode()

        #opens the gripper
        self.open_grip()

        #runs program until the game is over
        while not board.is_game_over() and not rospy.is_shutdown():
            try:
                if switch == 0:
                    #chess turn for human
                    self.to_hovering()
                    print("Waiting for key input")

                    #reads saved last board position from txt file
                    with open("current_board.txt", 'r') as f:
                        data = f.readlines()
                    lastline = data[-1]
                    board = chess.Board(fen=lastline)
                    f.close

                    #get current position of chess board after human move. SHOULD be taken after human move, or else cause error
                    current_grid = OpenCvRealSenseCameras.test()

                    #using chess_calculate code I wrote, track the human movement in UCI movement form ex)a1a4
                    move = chess_calculate.get_move(board.fen(), current_grid)
                    print(move)

                    #check whether right side is playing
                    if board.turn <> True:
                        print("Something is wrong, white is player")
                        break

                    #put the last saved chess board position inside the engine
                    engine.position(board)

                    #do the next move inside the data chessboard, so that it tracks the human movement
                    board.push(move)
                    print(board)

                    #saves the human movement in txt
                    f = open("current_board.txt", 'a')
                    data = board.fen()
                    f.write(data+"\n")
                    f.close
                    print("file saved")

                    time.sleep(5)
                    print('blue turn to move')
                    switch = 1

                elif switch == 1:

                    #robot turn to move
                    print("Moving in 3 seconds")

                    #read the last position of the chessboard
                    with open("current_board.txt", 'r') as f:
                        data = f.readlines()
                    lastline = data[-1]
                    board = chess.Board(fen=lastline)
                    f.close

                    #check whether right side is playing
                    if board.turn == True:
                        print("Something is wrong, white is player")
                        break

                    engine.position(board)
                    #using engine.go, you get the best next movement according to the given calculation time in ms
                    move = engine.go(movetime=1000)

                    #check whether next move is a capture
                    capture_status = board.is_capture(move.bestmove)

                    #put the next move in data chessboard
                    board.push(move.bestmove)
                    print(board)
                    print(move.bestmove)
                    time.sleep(3)

                    #if next move is capture, use move_capture to first remove the captured block. Else, just move ordinary.
                    if capture_status == True and board.turn == True:
                        self.move_capture(str(move.bestmove), 'w', num_w)
                        num_w += 1
                    elif capture_status == True and board.turn == False:
                        self.move_capture(str(move.bestmove), 'b', num_b)
                        num_b += 1
                    else:
                        self.move_ordinary(str(move.bestmove))

                    #save current status
                    f = open("current_board.txt", 'a')
                    data = board.fen()
                    f.write(data+"\n")
                    f.close
                    print("file saved")

                    print('Red turn to move')
                    switch = 0

                elif switch == 2:
                    #reset all the status to start a new game
                    print("initializing the chess board")
                    f = open("current_board.txt", 'w')
                    data = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w - - 0 1\n"
                    f.write(data)
                    f.close
                    switch = 0
                elif switch == 3:
                    #checking stage if you are trying to restart a game.
                    self.to_hovering()
                    print("please check the board position before it starts")
                    with open("current_board.txt", 'r') as f:
                        data = f.readlines()
                    lastline = data[-1]
                    board = chess.Board(fen=lastline)
                    num_b = 16
                    num_w = 16
                    for i in range(len(lastline)):
                        if lastline[i] == " ":
                            break
                        elif lastline[i].isupper()==True:
                            num_w -= 1
                        elif lastline[i].islower()==True:
                            num_b -= 1         

                    #shows board so that you can compare with the physical board. Also shows number of each dead pieces.
                    print(board)
                    print("blue Dead"+ str(num_b))
                    print("red Dead"+ str(num_w))
                    if board.turn == True:
                        print('red turn to move')
                    else: 
                        print('blue turn to move')
                    #10 seconds to check
                    time.sleep(10)
                    if board.turn == True:
                        switch = 0 
                    else:
                        switch = 1

            except KeyboardInterrupt:
                self.coordinator.close()
                print("interrupting")
                break
        print("the program ended!")
        self.coordinator.close()

if __name__ == "__main__":
    cp = ChessPlanner()
    cp.run()
