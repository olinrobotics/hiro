#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import pandas as pd
import random
import time
from std_msgs.msg import String, Int16
import cPickle as pickle
from game_board import C4Board
from minimax import Minimax

"""
The Connect 4 Game module
by Kevin Zhang and Hannah Kolano

to run you must connect to Draco, the dragon. see github for how to do that

also requires the following to be run:
1. roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=10.42.0.175
2. rosrun rosserial_python serial_node.py /dev/ttyACM0
3. rosrun irl ur5_arm_node.py

then run this script

basic steps:
1. plays connect 4, will always go second
2. waits for an input for a human, then calculates and executes its own move
3. repeats until a winner is found
"""


class C4(object):
    """
    the Connect 4 game, with computer vision for understanding the game board and
    whose turn it is, minimax for planning its own moves, and basic control system
    to make those moves and other interactions
    """

    def __init__(self, render=True, ros_node=None):
        self.board = C4Board(render)
        self.max = Minimax()

        if not ros_node:
            rospy.init_node("connect4", anonymous=True)

        self.ur5_commander = rospy.Publisher("behaviors_cmd", String, queue_size=10)
        self.token_grabber = rospy.Publisher("gripper", Int16, queue_size=10)
        self.computer_vision = rospy.Publisher("c4_ready", Int16, queue_size=10)
        rospy.Subscriber("arm_status", String, self.status_callback)
        rospy.Subscriber("opponent_move", Int16, self.player_move)

        self.status = True
        self.turn = "Start"
        self.player_action = None


    def player_move(self, data):
        """
        callback from Hannah's CV module, gives back an integer indicating which
        column the player moved to
        """

        self.player_action = data.data
        self.turn = "HUMAN"


    def status_callback(self, data):
        """
        callback for arm status
        """

        if data.data == "free":
            self.status = True
        else:
            self.status = False


    def check_for_completion(self):
        """
        checks to make sure that the arm is done with the previous command
        before continuing
        """

        time.sleep(3)
        while not self.status:
            pass


    def ur5_publish(self, command):
        """
        publishes a command, then checks for completion of said command before
        continuing the script
        """

        self.ur5_commander.publish(command)
        self.check_for_completion()


    def control_mouth(self):
        """
        controls the mouth, opens (0), waits a bit, then closes (1)
        """

        self.token_grabber.publish(0)
        time.sleep(3)
        self.token_grabber.publish(1)


    def make_move(self, move):
        """
        makes a physical move with the physical board, which includes getting a token,
        then moving to drop it into the chosen column, then returning to home position
        """

        # looks for token
        self.ur5_publish("c4_load_token")
        self.control_mouth()

        # makes a move and drops token
        self.ur5_publish("c4_move_"+str(move))
        self.control_mouth()
        self.ur5_publish("c4_home")


    def play_game(self):
        """
        plays a single game of connect 4
        """

        self.board.reset()

        while True:

            # waits for opponent move from rostopic
            print "WAITING FOR OPPONENT'S MOVE"
            stuff = raw_input("Move made?")
            self.computer_vision.publish(1)
            while self.turn != "HUMAN":
                pass

            # self.computer_vision.publish(0)

            # updates its own digital board
            observation_, done = self.board.step(self.player_action, 1)

            # break if human wins
            if done:
                print "\n"*2
                print "HUMAN WINS"
                return "HUMAN"
            elif done == "Draw":
                print "\n"*2
                print "DRAW"
                return "HUMAN"

            # AI makes moves and updates digital board
            self.turn = "AI"
            move, value = self.max.bestMove(5, observation_)
            _, done = self.board.step(move, 2)

            print "I MOVED TO COLUMN", str(move+1)

            # AI physically moves
            self.make_move(move+1)
            self.computer_vision.publish(1)

            # break if ai wins
            if done:
                print "\n"*2
                print "AI WINS"
                return "AI"
            elif done == "Draw":
                print "\n"*2
                print "DRAW"
                return "HUMAN"



    def run(self):
        """
        main interaction sequence
        """

        print "Starting up Connect 4 game"
        time.sleep(3)
        self.ur5_publish("c4_start")
        self.ur5_publish("c4_home")
        self.computer_vision.publish(1)
        time.sleep(1)
        result = self.play_game()

        if result == "AI":
            command = "c4_win"
        else:
            command = "c4_loss"

        self.ur5_publish(command)
        self.ur5_publish("c4_end")


if __name__ == "__main__":
    connect = C4()
    connect.run()
