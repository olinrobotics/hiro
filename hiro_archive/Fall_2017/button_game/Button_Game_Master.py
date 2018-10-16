#!/usr/bin/env python
import os
import argparse
import numpy as np
import cv2
import rospy
from std_msgs.msg import String
from irl.srv import arm_cmd
import math
import random
import time


class SimonSays():
    def __init__(self):
        print("INIT | Initializing")
        self.debug = True;
        #track the number of buttons before starting the game
        self.num_Buttons = 0;
        #total length of sequence
        self.sequence_length = 5
        #array for storing the correct sequence of colors
        rospy.Subscriber('arm_status', String, self.status_callback, queue_size = 10)
        self.status = 1
        self.sequence = np.zeros(self.sequence_length);
        print(self.sequence)
        #current element of the simon says sequence
        self.current_number = 0
        #current color press detected by edwin.
        # -1 is no color
        self.current_color = -1
        self.looking = False

        #Current state of the game
        #0 = in progress
        #1 = game won
        #2 = game lost
        self.game_state = 0
        self.start_time = 0

        #publisher for node to move arm to specific coordinate
        self.node_publish = rospy.Publisher('/arm_cmd', String, queue_size = 10)
        rospy.init_node('commands', anonymous=True)
        #subscriber for LED feedback script
        self.push_sub = rospy.Subscriber("chatter", String, self.push_sub)
        #publisher for node to execute a specfic behavior
        self.behavior_publish = rospy.Publisher('behaviors_cmd', String, queue_size = 10)

        #sends the home route to other node
        #self.publish(go_home)

        #actually returns edwin to the home position
        self.red_coor_high = "move_to:: 1000, 3600, 1100, 0"
        self.red_coor_low = "move_to:: 1000, 3600, -1100, 0"
        #self.yellow_coor_high = "move_to:: 1900, 6500, 3200, 0"
        #self.yellow_coor_low = "move_to:: 1900, 6500, -1100, 0"
        self.green_coor_high = "move_to:: -1100, 3600, 1100, 0"
        self.green_coor_low = "move_to:: -1100, 3600, -1100, 0"
        self.blue_coor_high = "move_to:: 0, 6500, 1100, 0"
        self.blue_coor_low = "move_to:: 0, 6500, -1100, 21"
        self.home = "move_to:: 0, 4850, 1100, 0"


        #generates the simon says sequence
        self.sequence = self.generate(self.sequence)
        print(self.sequence)
        print(self.current_color)
        self.request_cmd(self.home)
        self.check_completion()
        self.request_cmd("rotate_hand:: 2000")
        self.request_cmd("rotate_wrist:: 2600")
        time.sleep(3)
        self.edwin_turn(self.sequence, self.current_number)

    def run(self):
        r = rospy.Rate(10)
        if self.game_state == 1:
            print("Game Won!")
        else:
            print("Game Lost")
        while not rospy.is_shutdown():
            r.sleep()

    def status_callback(self, data):
        print "Writing arm status callback", data.data
        if data.data == "busy" or data.data == "error":
            self.status = 0
        elif data.data == "free":
            self.status = 1
    def request_cmd(self, cmd):
        self.check_completion()
        rospy.wait_for_service('arm_cmd', timeout=15)
        cmd_fnc = rospy.ServiceProxy('arm_cmd', arm_cmd)
        print "I have requested the command"

        try:
            resp1 = cmd_fnc(cmd)
            print "Command done"

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def check_completion(self):
        """
        Makes sure that actions run in order by waiting for response from service
        """
        time.sleep(1)
        # r = rospy.Rate(10)
        while self.status == 0:
            # r.sleep()
            pass
    '''
    def request_cmd(self, coor):
        """
        This method runs a specific route for edwin
        """
        command = coor
        print("RUN | Sending:", command)
        self.node_publish.publish(command)
        time.sleep(2)
    '''
    def generate(self, array):
        """
        This method generates an array with 30 elements, each a random color,
        either red, yellow, green or blue. This is the array that edwin uses
        to play with the user.
        """
        for i in range(self.sequence_length):
            color = random.randint(1, 3)
            print(color)
            array[i] = color
            #red = 1
            #green = 2
            #blue = 3
        return array

    def push_sub(self, data):
        """
        This method detects wheter a button has been pressed. If it has, then
        it returns the color of the button pressed.
        """
        if self.looking:
            print(data.data)
            if data.data == "Red on":
                self.current_color = 1
            elif data.data == "Green on":
                self.current_color =  2
            elif data.data == "Blue on":
                self.current_color = 3
        else:
            self.current_color = -1

    def edwin_turn(self, array, position):
        """
        This method is what edwin will execute when it is his turn. He will
        locate the buttons and press them in the right order the right number
        of times.
        """
        print("edwin's turn!")
        self.looking = False
        self.current_color = -1
        #return home
        self.request_cmd(self.home)
        #Check to see if the player won
        if (position + 1 == len(array)):
            self.game_state = 1
            self.behavior_publish.publish("praise")
            return None

        #cycle through the color array until the current position
        for color in array[:position]:
            if color == 1:
                self.request_cmd(self.red_coor_high)
                self.request_cmd(self.red_coor_low)
            elif color == 2:
                self.request_cmd(self.green_coor_high)
                self.request_cmd(self.green_coor_low)
            elif color == 3:
                self.request_cmd(self.blue_coor_high)
                self.request_cmd(self.blue_coor_low)
            time.sleep(1)
            self.request_cmd(self.home)

            #self.start_time = time.time()
        self.player_turn(array, position)


    def player_turn(self, array, position):
        """
        This method makes edwin watch the user's inputs to make sure they're valid
        """
        print("player's turn!")
        self.looking = True
        for color in array[:position]:
            #check to see that all buttons are in frame
            #elapsed_time = time.time()
            #while (elapsed_time - 10 < self.start_time)
            #wait until a button is pressed
            while (self.current_color == -1):
                time.sleep(0.01)
            #React angriy if the wrong button is pressed and end the game.
            if (self.current_color != color):
                self.behavior_publish.publish("angry")
                self.game_state = 2
                return None
            time.sleep(0.5)
            self.current_color = -1
                #elapsed_time = time.time()

        #by now, the user must have pressed the correct button. add one to the
        #current position array, and then call edwin_turn
        self.edwin_turn(array, position + 1)
if __name__ == "__main__":
    print("hello")
    button_game = SimonSays()
    button_game.run()
