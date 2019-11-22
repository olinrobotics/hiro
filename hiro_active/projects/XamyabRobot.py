#!/usr/bin/env python
"""
A class to represent Xamyab robot. It contains all the Xamyab's move groups
and provides basic functionalities to control them.

Manipulator move groups:
- Able to home the left/right manipulator
- Able to set joint values
- Able to set pose values

Gripper move groups:
- Allow the left/right gripper to open and close
- Able to set a specific value for the gripper
"""
from geometry_msgs.msg import Quaternion, Point
from tf.transformations import quaternion_from_euler
from hiro_core.GenericRobot import *


class XamyabRobot(GenericRobot):
    def __init__(self, *args, **kwargs):
        super(XamyabRobot, self).__init__(*args, **kwargs)
        self._setup_movegroup_commanders()

    def _setup_movegroup_commanders(self):
        """
        Set up useful functions for manipulators and grippers
        :return: None
        """
        # Init commanders
        self.left_manipulator = self.move_groups["left_manipulator"]
        self.right_manipulator = self.move_groups["right_manipulator"]
        self.left_gripper = self.move_groups["left_gripper"]
        self.right_gripper = self.move_groups["right_gripper"]

        # Assign set default state function for each commander
        self.left_manipulator.set_default_state = lambda state: self.set_default_state(self.left_manipulator, state)
        self.right_manipulator.set_default_state = lambda state: self.set_default_state(self.right_manipulator, state)
        self.left_gripper.set_default_state = lambda state: self.set_default_state(self.left_gripper, state)
        self.right_gripper.set_default_state = lambda state: self.set_default_state(self.right_gripper, state)

        # Set up some default goal states
        self.left_manipulator.home = lambda: self.left_manipulator.set_default_state("left_home")
        self.right_manipulator.home = lambda: self.right_manipulator.set_default_state("right_home")
        self.left_gripper.close = lambda: self.left_gripper.set_default_state("lgripper_close")
        self.right_gripper.close = lambda: self.right_gripper.set_default_state("rgripper_close")
        self.left_gripper.open = lambda: self.left_gripper.set_default_state("lgripper_open")
        self.right_gripper.open = lambda: self.right_gripper.set_default_state("rgripper_open")

        # Assign set pose goal function for each manipulator
        self.left_manipulator.set_pose_goal = lambda goal: self.set_pose_goal(self.left_manipulator, goal)
        self.right_manipulator.set_pose_goal = lambda goal: self.set_pose_goal(self.right_manipulator, goal)

        # Assign set update joint values function for each manipulator
        self.left_manipulator.set_joint_values = lambda goal: self.set_joint_values(self.left_manipulator, goal)
        self.right_manipulator.set_joint_values = lambda goal: self.set_joint_values(self.right_manipulator, goal)

        # Assign set value function for each gripper
        self.left_gripper.set_value = lambda value: self.set_gripper_value(self.left_gripper, value)
        self.right_gripper.set_value = lambda value: self.set_gripper_value(self.right_gripper, value)

    def set_gripper_value(self, gripper_group, value):
        # type: (str or moveit_commander.MoveGroupCommander, float) -> bool
        """
        Set value to adjust the gripper.
        :param gripper_group: left_gripper or right_gripper move group name or commander
        :param value: between 0% (open) and 100% (close)
        :return: bool: is successful
        """
        # Squash value between 0% (open) and 100% (close)
        if value < 0 or value > 100:
            return False

        value = value * 0.68 / 100
        return self.set_joint_values(gripper_group, [value, value])

    def test(self):
        q = quaternion_from_euler(pi, 0, 0)
        april_4 = geometry_msgs.msg.Pose(position=Point(*[0.6256, 0, 0.4]), orientation=Quaternion(*q))
        april_1 = geometry_msgs.msg.Pose(position=Point(*[0.5124, -0.24, 0.4]), orientation=Quaternion(*q))
        april_0 = geometry_msgs.msg.Pose(position=Point(*[0.5124, 0.24, 0.4]), orientation=Quaternion(*q))

        # Test left manipulator
        self.left_manipulator.home()
        self.left_manipulator.set_pose_goal(april_4)
        self.left_manipulator.set_pose_goal(april_1)
        self.left_manipulator.set_pose_goal(april_0)
        self.left_manipulator.home()

        # Test left gripper
        self.left_gripper.set_value(50)
        self.left_gripper.set_value(100)
        self.left_gripper.set_value(0)
        self.left_gripper.close()
        self.left_gripper.open()

        # Test right manipulator
        self.right_manipulator.home()
        self.right_manipulator.set_pose_goal(april_4)
        self.right_manipulator.set_pose_goal(april_1)
        self.right_manipulator.set_pose_goal(april_0)
        self.right_manipulator.home()

        # Test right gripper
        self.right_gripper.set_value(50)
        self.right_gripper.set_value(100)
        self.right_gripper.set_value(0)
        self.right_gripper.close()
        self.right_gripper.open()


if __name__ == '__main__':
    robot = XamyabRobot(visualize_trajectory=False)
    robot.test()
