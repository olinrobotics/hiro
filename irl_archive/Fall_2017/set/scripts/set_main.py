"""
By Khang Vu, Cassandra Overney & Enmo Ren, 2017
Last modified Dec 17, 2017

This script is the master script of the Set Game
Currently works for a 3x4 Set board

Dependencies:
- Turn.py
- opencv.py
- USB Camera

To use:
- Run the code below

roscore
rosrun irl edwin_node.py
rosrun irl edwin_routes.py
rosrun irl edwin_behaviors.py
rosrun usb_cam usb_cam_node _video_device:='/dev/video1' _image_width:=1280 _image_height:=720
rosrun irl set_main.py
"""

import time

import numpy as np

import cv2
import rospy
from cv_bridge import CvBridgeError, CvBridge
from irl.srv import arm_cmd
from sensor_msgs.msg import Image
from std_msgs.msg import String

from Turn import *
from opencv import CEO


class SetMain(object):
    """
    Master class of the game Set
    """

    def __init__(self, ros_node=None):
        """
        This init node, subscribers and publishers
        :param ros_node:
        """
        self.route_string = 'R_set_center'

        # init ROS nodes
        if not ros_node:
            rospy.init_node('set_gamemaster', anonymous=True)

        # init ROS subscribers to camera and status
        rospy.Subscriber('arm_status', String, self.status_callback, queue_size=10)
        rospy.Subscriber('usb_cam/image_raw', Image, self.img_callback)

        self.behavior_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)

        # For the image
        self.bridge = CvBridge()

        # Edwin's status: 0 = busy, 1 = free
        self.status = 1

        # Video frame
        self.frame = None

        # x, y, z positions of Edwin
        self.x = 0
        self.y = 0
        self.z = 0

        # Captured image from the camera
        self.set_image = None

        # Result: a set
        self.result = []

        # Whether we're using a suction cup
        self.use_suction_cup = True

    def status_callback(self, data):
        """
        Get status from arm_node
        :param data: status callback
        :return: None
        """
        print "Arm status callback", data.data
        if data.data == "busy" or data.data == "error":
            self.status = 0
        elif data.data == "free":
            self.status = 1

    def img_callback(self, data):
        """
        Get image from usb camera
        :param data: image
        :return: None
        """
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

    def request_cmd(self, cmd):
        self.check_completion()
        rospy.wait_for_service('arm_cmd', timeout=15)
        cmd_fnc = rospy.ServiceProxy('arm_cmd', arm_cmd)
        print "I have requested the command"

        try:
            cmd_fnc(cmd)
            print "Command done"

        except rospy.ServiceException, e:
            print ("Service call failed: %s" % e)

    def check_completion(self, duration=1.0):
        """
        Makes sure that actions run in order by waiting for response from service
        """
        time.sleep(duration)
        r = rospy.Rate(10)
        while self.status == 0:
            r.sleep()
            pass

    def route_move(self, route_string):
        msg = "data: run_route:: " + route_string
        print ("sending: ", msg)
        self.request_cmd(msg)

    def speed_set(self, num):
        msg = "data: set_speed:: " + str(num)
        print ("sending: ", msg)
        self.request_cmd(msg)

    def move_xyz(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        msg = "data: move_to:: %i, %i, %i, %i" % (self.x, self.y, self.z, 0)
        print ("Sending", msg)
        self.request_cmd(msg)

    def move_wrist(self, value):
        msg = "data: rotate_wrist:: " + str(value)
        print ("sending: ", msg)
        self.request_cmd(msg)

    def move_hand(self, value):
        msg = "data: rotate_hand:: " + str(value)
        print ("sending: ", msg)
        self.request_cmd(msg)

    def behave_move(self, behavior_string):
        msg = behavior_string
        print ("sending: ", msg)
        self.behavior_pub.publish(msg)

    def move_head(self, hand_value=None, wrist_value=None):
        """
        Always move hand first, wrist second
        :param hand_value:
        :param wrist_value:
        :return: None
        """
        self.move_hand(hand_value)
        self.move_wrist(wrist_value)

    def move_to_center(self):
        """
        Move edwin to the center position where it can take a good picture
        :return: None
        """
        self.move_xyz(x=0, y=3500, z=2000)
        self.move_head(hand_value=1900, wrist_value=2500)

    def move_to_center_route(self):
        """
        Move edwin to the center position where it can take a good picture
        Using routes
        :return: None
        """
        self.route_move(self.route_string)

    def capture_piture(self):
        """
        Capture picture from usb_cam and pass it to self.set_image
        :return: None
        """
        self.check_completion()
        r = rospy.Rate(10)
        while self.frame is None:
            r.sleep()
            pass

        self.set_image = self.frame.astype(np.uint8)
        cv2.imshow('Image', self.set_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def capture_video(self):
        """
        Capture video from usb_cam
        :return: None
        """
        self.check_completion()
        r = rospy.Rate(10)
        while self.frame is None:
            r.sleep()
            pass

        while self.frame is not None:
            r.sleep()
            cv2.imshow('Image', self.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def pout_behavior(self):
        """
        Make Edwin do sad behavior to show that there is no Set
        :return:
        """
        self.behave_move("pout")

    def pick_card(self, row, col):
        """
        Move the arm to (row, col) and pick up the card
        :return: None
        """
        print "row = %i, col = %i" % (row, col)
        x, y, z = self.get_coordinates_3_by_4(row, col)
        self.move_xyz(x, y, z - 680)
        return x, y, z

    def pick_cards(self):
        """
        Pick a set (3 cards) from the board
        :return: None
        """
        if self.use_suction_cup:
            for set_card in self.result:
                row, col = set_card.coord
                self.move_xyz(x=0, y=3500, z=2000)
                x, y, z = self.pick_card(row, col)
                if not self.continue_or_not():
                    self.move_xyz(x=0, y=3500, z=2000)
                    break
                else:
                    self.move_xyz(x=x, y=y, z=z - 480)
                    self.move_to_stack()
                    if not self.continue_or_not():
                        break
        else:
            for set_card in self.result:
                row, col = set_card.coord
                self.pick_card(row, col)
                if not self.continue_or_not():
                    self.move_to_center()
                    break

    def move_to_stack(self):
        """
        Move a card to a stack out of the board
        :return:
        """
        self.move_xyz(x=-2700, y=4500, z=-680)

    def test_pick_cards(self):
        """
        Test function to pick cards on the 3 x 4 board
        :return: None
        """
        for i in range(4):
            for j in range(3):
                self.pick_card(i, j)

    def continue_or_not(self):
        """
        Function asks the users if they want to continue the program.
        :return: True to continue; False otherwise
        """
        self.check_completion(0.5)
        while True:
            answer = raw_input("Do you want me to continue (yes/no)? ").lower()
            if "y" in answer:
                print "Continue"
                return True
            if "n" in answer:
                print "Stopped"
                return False
            else:
                print "Invalid answer"

    def suction_cup_or_not(self):
        """
        Function asks the users if they want to continue the program.
        :return: True to continue; False otherwise
        """
        self.check_completion(0.5)
        while True:
            answer = raw_input("Are you using a suction cup (yes/no)? ").lower()
            if "y" in answer:
                print "Use suction cup"
                self.use_suction_cup = True
                break
            elif "n" in answer:
                print "Do not use suction cup"
                self.use_suction_cup = False
                break
            else:
                print "Invalid answer"

    def play_again(self):
        """
        Function asks the users if they want to play the game again.
        :return: True to continue; False otherwise
        """
        self.check_completion(0.5)
        while True:
            answer = raw_input("Do you want to play again (yes/no)? ").lower()
            if "y" in answer:
                print "Re-play"
                return True
            elif "n" in answer:
                print "Game ended"
                return False
            else:
                print "Invalid answer"

    def run(self):
        """
        Main function that runs everything
        :return: None
        """
        while True:
            # Speed set to 1000
            self.speed_set(1000)

            # Move Edwin to center position
            self.move_to_center_route()

            # Asking if using the suction cup
            self.suction_cup_or_not()

            # Capture the image
            self.capture_piture()

            # Ask if users want to continue (Does the captured image look right?)
            if not self.continue_or_not():
                if self.play_again():
                    continue
                else:
                    break

            # Try to find a set
            try:
                ceo = CEO()
                all_cards = ceo.find_matches(self.set_image)
                turn = Turn(all_cards)
                self.result = turn.find_set()

                # Check if everything is alright
                turn.print_card_array(turn.card_array)

                # If there is no result
                if not self.result:
                    self.pout_behavior()
                    self.move_to_center()
                # Else go pick the cards
                else:
                    self.pick_cards()

                # Move back to center after picking the cards
                self.move_to_center()
                if not self.play_again():
                    break
            except:
                print("Something is wrong. Please take another picture!")
                self.play_again()

        # End game behavior
        self.behave_move("done_game")

        # Make sure the speed is set back to 1000
        self.speed_set(1000)

    def get_coordinates_3_by_4(self, row, col):
        """
        Get coordinates x, y, z from row, col
        Only applicable for 3 x 4 board
        :param row: 0 to 2
        :param col: 0 to 3
        :return: x, y, z
        """
        if row == 0 and col == 0:
            x = -700
            y = 5800
            z = 0

        elif row == 0 and col == 1:
            x = 400
            y = 5800
            z = 0

        elif row == 0 and col == 2:
            x = 1400
            y = 5800
            z = 0

        elif row == 1 and col == 0:
            x = -700
            y = 5000
            z = 0

        elif row == 1 and col == 1:
            #
            x = 400
            y = 5000
            z = 0

        elif row == 1 and col == 2:
            x = 1400
            y = 5000
            z = 0

        elif row == 2 and col == 0:
            x = -700
            y = 4300
            z = 0

        elif row == 2 and col == 1:
            x = 400
            y = 4300
            z = 0

        elif row == 2 and col == 2:
            x = 1400
            y = 4300
            z = 0

        elif row == 3 and col == 0:
            x = -700
            y = 3600
            z = 0

        elif row == 3 and col == 1:
            x = 400
            y = 3600
            z = 0

        elif row == 3 and col == 2:
            x = 1400
            y = 3600
            z = 0

        else:
            x = 0
            y = 3400
            z = 4700

        return x, y, z


if __name__ == '__main__':
    set_game = SetMain()
    set_game.run()
