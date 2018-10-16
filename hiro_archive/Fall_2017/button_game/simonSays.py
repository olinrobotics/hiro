import numpy as np
import cv2
import rospy
import math
import random
import time
from cv_bridge import CvBridge, CvBridgeError

class SimonSays:
    def __init__(self, init=False):
        print("INIT | Initializing")
        self.debug = True;
        #track the number of buttons before starting the game
        self.num_Buttons = 0;
        #array for storing the correct sequence of colors
        self.sequence = np.zeros(30);
        #current element of the simon says sequence
        self.current_number = 0;
        #Current state of the game
        #0 = in progress
        #1 = game won
        #2 = game lost
        self.game_state = 0
        self.start_time = 0

        #publisher for node to move arm to specific coordinate
        self.node_publish = rospy.Publisher('/arm_cmd', String, queue_size = 10)
        #publisher for node to execute a specfic behavior
        self.behavior_publish = rospy.Publisher('behaviors_cmd', String, queue_size = 10)


        #returns edwin to the home position
        go_home = "create_route:: home; 0, 4000, 4000, 767, 195, 0"
        print("INIT | Going home")

        #sends the home route to other node
        self.publish(go_home)
        self.bridge = CvBridge()

        #Do I need this?
        self.image = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)

        #actually returns edwin to the home position
        self.red_coor = "move_to:: #x, #y, #z, #pitch"
        self.yello_coor = "move_to:: #x, #y, #z, #pitch"
        self.green_coor = "move_to:: #x, #y, #z, #pitch"
        self.blue_coor = "move_to:: #x, #y, #z, #pitch"
        self.home = "move_to:: #x, #y, #z, #pitch"

        #generates the simon says sequence
        self.sequence = self.generate(self.sequence)

        self.edwin_turn(self.sequence, self.current_number)

    def move_coor(self, coor):
        """
        This method runs a specific route for edwin
        """
        command = coor
        print("RUN | Sending:", command)
        self.node_publish.publish(command)
        time.sleep(2)

    def generate(self, array):
        """
        This method generates an array with 30 elements, each a random color,
        either red, yellow, green or blue. This is the array that edwin uses
        to play with the user.
        """
        for i in Range(30):
            color = randint(1, 4)
            if (color == 1):
                array[i] = "red"
            elif (color == 2):
                array[i] = "yellow"
            elif(color == 3):
                array[i] = "green"
            else:
                array[i] = "blue"

    '''
    Depreciated. Buttons are no longer in a random position.

    def calc_coordinates(self, color):
        """
        This method looks for a specific color of button from the camera. It
        then calculates the coordinates in terms of edwin's coordinate system
        and then returns those coordinates as a string
        """
        if (self.num_Buttons != 4):
            return "ERROR"
        else:

            #return coordinates from open CV after conversion
    '''

    def detect_press(self):
        """
        This method detects wheter a button has been pressed. If it has, then
        it returns the color of the button pressed. If it hasn't, then it returns false
        """

    def edwin_turn(self, array, position):
        """
        This method is what edwin will execute when it is his turn. He will
        locate the buttons and press them in the right order the right number
        of times.


        """
        #return home
        self.move_coor("home")

        #Check to see if the player won
        if (position == len(array)):
            self.game_state = 1
            self.behavior_publish("praise")
            return None

        #cycle through the color array until the current position
        for color in array[:position]:
            #If not all the buttons are in view, nod
            while (self.calc_coordinates(color = color) == "ERROR"):
                self.behavior_publish("nod")
                time.sleep(1)

            #Move to the coordinate, and then return home
            move_command = self.calc_coordinates(color = color)
            self.node_publish(move_command)
            time.sleep(1)
            self.run_route("home")

            self.start_time = time.time()
            self.player_turn(array, position)


    def player_turn(self, array, position):
        """
        This method makes edwin watch the user's inputs to make sure they're valid
        """


        for color in array[:position]:
            #check to see that all buttons are in frame
            elapsed_time = time.time()
            while (self.calc_coordinates(color = color) == "ERROR"):
                self.behavior_publish("nod")
                time.sleep(1)
            while (elapsed_time - 10 < self.start_time)
                #wait until a button is pressed
                while (!self.detect_press):
                    time.sleep(0.1)
                #React angriy if the wrong button is pressed and end the game.
                if (self.detect_press != array[color]):
                    self.behavior_publish("angry")
                    self.game_state = 2
                    return None
                elapsed_time = time.time()

        #by now, the user must have pressed the correct button. add one to the
        #current position array, and then call edwin_turn
        self.edwin_turn(array, position + 1)
