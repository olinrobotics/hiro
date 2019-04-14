"""
Controls how the gripper moves
"""
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
import time


class GripperController():
    def __init__(self):
        rospy.init_node('robotiq_gripper_controller', anonymous=True)
        self.pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output,
                                   queue_size=10)
        self.command = outputMsg.Robotiq2FGripper_robot_output();

        # activate the gripper
        self.command = outputMsg.Robotiq2FGripper_robot_output();
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSP = 255
        self.command.rFR = 150
        self.pub.publish(self.command)
        time.sleep(1)

    def close(self):
        # close gripper / change its position to 255
        self.command.rPR = 255
        self.pub.publish(self.command)
        time.sleep(1)

    def open(self):
        self.command.rPR = 0
        self.pub.publish(self.command)
        time.sleep(1)

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.close()

                time.sleep(5)
            except KeyboardInterrupt:
                self.command = outputMsg.Robotiq2FGripper_robot_output();
                self.command.rACT = 0
                self.pub.publish(self.command)
                time.sleep(1)

                break


if __name__ == "__main__":
    gc = GripperController()
