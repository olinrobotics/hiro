#!/usr/bin/env python
from geometry_msgs.msg import Quaternion, Point
from tf.transformations import quaternion_from_euler
from hiro_core.GenericRobot import *


class XamyabRobot(GenericRobot):
    def __init__(self, *args, **kwargs):
        super(XamyabRobot, self).__init__(*args, **kwargs)


if __name__ == '__main__':
    robot = XamyabRobot(visualize_trajectory=True)
    pose_goal = geometry_msgs.msg.Pose()
    q = quaternion_from_euler(pi, 0, 0)
    pose_goal.orientation = Quaternion(*q)
    april_4 = [0.6256, 0, 0.4]
    april_1 = [0.5124, -0.24, 0.4]
    april_0 = [0.5124, 0.24, 0.4]
    pose_goal.position = Point(*april_1)
    robot.set_pose_goal('left_manipulator', pose_goal)
