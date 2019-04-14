"""
Defines arm gestures, controlling how the arms should move
"""
import time

import rospy
from std_msgs.msg import String


class DrawArm():
    """
    This is the code to move the UR5 arms for the Fall 18 HIRo project to draw portraits
    of celebrities using OCR.
    This module draws the portraits by moving the arm with a sharpie in the gripper to
    discrete xyz positions
    """

    def __init__(self):
        # rospy.init_node("draw_arm", anonymous=True)

        # --------CASTOR----------
        # self.name = 'castor'
        # self.coordinates_pub_castor = rospy.Publisher("/coordinates_cmd_castor", String, queue_size=10)
        # self.joints_pub_castor = rospy.Publisher("/behaviors_cmd_castor", String, queue_size=10)

        # --------POLLOX----------
        self.name = 'pollux'
        self.coordinates_pub_pollux = rospy.Publisher("/coordinates_cmd_pollux", String, queue_size=10)
        self.joints_pub_pollux = rospy.Publisher("/behaviors_cmd_pollux", String, queue_size=10)

        time.sleep(1)

    def move_gesture(self, msg):
        print("Sending: ", msg)
        # self.joints_pub_castor.publish(msg)
        self.joints_pub_pollux.publish(msg)
        print('Sent gesture')

    def move_coord(self, msg):
        print("Sending: ", msg)
        # self.coordinates_pub_castor.publish(msg)
        self.coordinates_pub_pollux.publish(msg)

    def draw_line(self, p1, p2):
        x1 = p1[0] / 1000.0
        y1 = p1[1] / 1000.0
        x2 = p2[0] / 1000.0
        y2 = p2[1] / 1000.0

        self.move_coord(str(x1) + ' ' + str(y1) + ' 0.305')
        self.move_coord(str(x2) + ' ' + str(y2) + ' 0.305')
        time.sleep(1)

    def run(self):
        print("Draw Arm running")
        while not rospy.is_shutdown():
            try:
                self.move_gesture('portrait_hover')
                time.sleep(5)
                return
            except KeyboardInterrupt:
                break


if __name__ == "__main__":
    draw = DrawArm()
    # draw.run()
