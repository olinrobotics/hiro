"""
By Sherrie Shen & Khang Vu, 2017
Last modified Dec 15, 2017

This script is the master script of the Sudoku Game.
Currently works for 4x4 Sudoku boards.

Dependencies:
- get_sudoku.py
- USB Camera

To use:
- Make sure you have font images in training_data/font_images. If not, read README to download the images.
- Then run the code below:

roscore
rosrun irl edwin_node.py
rosrun irl edwin_routes.py
rosrun irl edwin_behaviors.py
rosrun irl edwin_sudoku_write.py
rosrun usb_cam usb_cam_node _video_device:='/dev/video1' _image_width:=1280 _image_height:=720
rosrun irl sudoku_main.py
"""
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridgeError, CvBridge
from irl.msg import Edwin_Shape
from irl.srv import arm_cmd
from sensor_msgs.msg import Image
from std_msgs.msg import String

import get_sudoku


class SudokuMain(object):
    """
    Master class of the game Sudoku
    """

    def __init__(self, n=4, ros_node=None):
        """
        This init node, subscribers and publishers as well as a sudoku object
        :param n: 4 or 9
        :param ros_node: True to init a node
        """
        # init ROS nodes
        self.route_string = 'R_sudoku_center'
        self.z_offset = 5
        if not ros_node:
            rospy.init_node('sudoku_gamemaster', anonymous=True)

        # init ROS subscribers to camera, arm status and writing status
        rospy.Subscriber('arm_status', String, self.status_callback, queue_size=10)
        rospy.Subscriber('writing_status', String, self.writing_status_callback, queue_size=20)
        rospy.Subscriber('usb_cam/image_raw', Image, self.img_callback)

        self.write_pub = rospy.Publisher('write_cmd', Edwin_Shape, queue_size=10)
        self.behavior_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)

        # For the image
        self.bridge = CvBridge()

        # Edwin's status: 0 = busy, 1 = free
        self.status = 1
        self.writing_status = 1

        # Video frame
        self.frame = None

        # x, y, z positions of Edwin
        self.x = 0
        self.y = 0
        self.z = 0

        # Sudoku size, either 4 or 9
        self.n = n

        # Captured image from the camera
        self.sudoku_image = None

        # Sudoku object
        self.sudoku = None

    def reinit(self):
        """
        Reinit function to restart the game
        :return: None
        """
        # Edwin's status: 0 = busy, 1 = free
        self.status = 1
        self.writing_status = 1

        # Video frame
        self.frame = None

        # x, y, z positions of Edwin
        self.x = 0
        self.y = 0
        self.z = 0

        # Captured image from the camera
        self.sudoku_image = None

        # Sudoku object
        self.sudoku = None

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

    def writing_status_callback(self, data):
        """
        Get status from arm_write node
        :param data: status callback
        :return: None
        """
        print "writing status callback", data.data
        if data.data == "writing":
            print "Busy writing"
            self.writing_status = 0
        elif data.data == "done":
            print "Free"
            self.writing_status = 1

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

    def check_completion(self, duration=1.5):
        """
        Makes sure that actions run in order by waiting for response from service
        """
        time.sleep(duration)
        r = rospy.Rate(10)
        while self.status == 0 or self.writing_status == 0:
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
        self.move_xyz(x=0, y=3400, z=4700)
        self.move_head(hand_value=3350, wrist_value=4020)

    def move_to_center_route(self):
        """
        Move edwin to the center position where it can take a good picture
        Using routes
        :return: None
        """
        self.route_move(self.route_string)

    def capture_piture(self):
        """
        Capture picture from usb_cam and pass it to self.sudoku_image
        :return: None
        """
        self.check_completion()
        r = rospy.Rate(10)
        while self.frame is None:
            r.sleep()
            pass

        self.sudoku_image = self.frame.astype(np.uint8)
        cv2.imshow('Image', self.sudoku_image)
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

    def write_number(self, row, col, number):
        """
        Move the arm to (row, col) and write a number
        :return: None
        """
        if number == -1:
            return
        x, y, z = self.get_coordinates_4_by_4(row, col)

        # Move Edwin to the position
        self.move_xyz(x, y, z + 200)

        # Write the number
        data = Edwin_Shape(x=x, y=y, z=z - self.z_offset, shape=str(number))
        self.write_pub.publish(data)

        # Pick marker off paper
        self.move_xyz(x, y, z + 200)

    def write_numbers(self):
        """
        Write solution on the Sudoku board
        After writing a number, ask users if they want to continue writing. If not, break the loop
        :return: None
        """
        solution = self.sudoku.solution
        count = len(solution)
        for cell in solution:
            row, col, number = cell.get_rc_num()
            count -= 1
            print "row = %i, col = %i, num = %i" % (row, col, number)
            self.write_number(row, col, number)
            if count > 0 and not self.continue_or_not():
                self.move_to_center()
                break

    def test_write_numbers(self):
        """
        Test function to write numbers on the 4 x 4 board
        :return: None
        """
        for i in range(1, 4):
            for j in range(4):
                print "Writing", i, j
                self.write_number(i, j, 8)
                # if not self.continue_or_not():
                #     break

    def continue_or_not(self):
        """
        Function asks the users if they want to continue the program.
        There is a question asking if the users want to change the z_offset
        :return: True to continue; False otherwise
        """
        self.check_completion(0.5)
        while True:
            answer = raw_input("Do you want me to continue (yes/no)? ").lower()
            if "y" in answer:

                answer = raw_input(
                    "Current offset is %s. Do you want to change the z_offset (yes/no)? " % self.z_offset).lower()
                if "y" in answer:
                    while True:
                        answer = raw_input("New z_offset = ")
                        try:
                            self.z_offset = int(answer)
                            print self.z_offset
                            break
                        except ValueError:
                            # Handle the exception
                            print 'Invalid value. Please enter an integer.'
                print "Continue"
                return True
            elif "n" in answer:
                print "Stopped"
                return False
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
            # Reinit every time the game starts
            self.reinit()

            # Speed set to 1000
            self.speed_set(1000)

            # Move to center
            self.move_to_center_route()
            self.move_to_center()

            # Capture picture of the sudoku
            self.capture_piture()

            # Convert the picture to a sudoku object
            self.sudoku = get_sudoku.from_image(im=self.sudoku_image, n=self.n)

            # If everything is alreight, continue
            if self.continue_or_not():
                if self.sudoku.solution is not None:
                    # Print the solution
                    self.sudoku.print_sudoku()

                    # Tell Edwin to write the solution on the board. Can be canceled after writing one digit
                    self.write_numbers()

                    # After finishing writing the solution, move Edwin back to center
                    self.move_to_center()

            # Ask users if they want to play again, if not break the loop
            if not self.play_again():
                break

        # End game behavior
        self.behave_move("done_game")

        # Make sure the speed is set back to 1000
        self.speed_set(1000)

    def get_coordinates_4_by_4(self, row, col):
        """
        Get coordinates x, y, z from row, col
        Only applicable for 4 x 4 board
        :param row: 0 to 3
        :param col: 0 to 3
        :return: x, y, z
        """
        if row == 0 and col == 0:
            x = -1500
            y = 6500
            z = -780

        elif row == 0 and col == 1:
            x = -400
            y = 6500
            z = -770

        elif row == 0 and col == 2:
            x = 800
            y = 6500
            z = -770

        elif row == 0 and col == 3:
            x = 1900
            y = 6500
            z = -770

        elif row == 1 and col == 0:
            x = -1500
            y = 5400
            z = -770

        elif row == 1 and col == 1:
            x = -450
            y = 5400
            z = -770

        elif row == 1 and col == 2:
            x = 700
            y = 5400
            z = -768

        elif row == 1 and col == 3:
            x = 1900
            y = 5400
            z = -774

        elif row == 2 and col == 0:
            x = -1500
            y = 4200
            z = -758

        elif row == 2 and col == 1:
            x = -400
            y = 4300
            z = -755

        elif row == 2 and col == 2:
            x = 700
            y = 4300
            z = -753

        elif row == 2 and col == 3:
            x = 1900
            y = 4400
            z = -763

        elif row == 3 and col == 0:
            x = -1500
            y = 3200
            z = -735

        elif row == 3 and col == 1:
            x = -450
            y = 3200
            z = -738

        elif row == 3 and col == 2:
            x = 700
            y = 3200
            z = -744

        elif row == 3 and col == 3:
            x = 1900
            y = 3200
            z = -750

        else:
            x = 0
            y = 3400
            z = 4700

        return x, y, z


if __name__ == '__main__':
    sudoku_game = SudokuMain()
    sudoku_game.run()
