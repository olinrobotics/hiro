"""
THIS IS THE OLD METHOD TO CONTROL THE GRIPPER. THIS DOES NOT
CONNECT WITH MOVEIT, AND SO IT IS NOT RECOMMENDED TO RUN THIS FILE.
"""

# Example code for operating the robotiq 2f-140 Gripper

import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
import time

class GripperController():
    def __init__(self):
        rospy.init_node('robotiq_gripper_controller')
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

        self.run()

    def do_stuff(self):
        # you should replace this with your own sequence of actions
        # but for now the gripper closes and opens again indefinitely

        # close gripper / change its position to 255
        self.command.rPR = 255
        self.pub.publish(self.command)
        time.sleep(2)

        # open gripper / change its position to 0
        self.command.rPR = 0
        self.pub.publish(self.command)
        time.sleep(2)

    def run(self):
        while not rospy.is_shutdown():
            try:
                # perform gripper functions multiple times
                self.do_stuff()

                time.sleep(5)
            except KeyboardInterrupt:
                # reset the gripper
                self.command = outputMsg.Robotiq2FGripper_robot_output();
                self.command.rACT = 0
                self.pub.publish(self.command)
                time.sleep(1)

                break

if __name__ == "__main__":
    gc = GripperController()
