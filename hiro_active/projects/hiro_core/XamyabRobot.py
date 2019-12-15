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

from copy import deepcopy

from geometry_msgs.msg import Quaternion, Point
from moveit_msgs.msg import PlanningSceneComponents
from tf.transformations import quaternion_from_euler

from GenericRobot import *


class XamyabRobot(GenericRobot):
    def __init__(self, *args, **kwargs):
        super(XamyabRobot, self).__init__(*args, **kwargs)
        self._setup_move_group_commanders()

    def _setup_move_group_commanders(self):
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
        self.left_manipulator._set_default_state = lambda state: self._set_default_state(self.left_manipulator, state)
        self.right_manipulator._set_default_state = lambda state: self._set_default_state(self.right_manipulator, state)
        self.left_gripper._set_default_state = lambda state: self._set_default_state(self.left_gripper, state)
        self.right_gripper._set_default_state = lambda state: self._set_default_state(self.right_gripper, state)

        # Set up some default goal states
        self.left_manipulator.home = lambda: self.left_manipulator._set_default_state("left_home")
        self.right_manipulator.home = lambda: self.right_manipulator._set_default_state("right_home")
        self.left_gripper.close = lambda: self.left_gripper._set_default_state("lgripper_close")
        self.right_gripper.close = lambda: self.right_gripper._set_default_state("rgripper_close")
        self.left_gripper.open = lambda: self.left_gripper._set_default_state("lgripper_open")
        self.right_gripper.open = lambda: self.right_gripper._set_default_state("rgripper_open")

        # Assign set pose goal function for each manipulator
        self.left_manipulator.set_pose_goal = lambda goal: self.set_pose_goal(self.left_manipulator, goal)
        self.right_manipulator.set_pose_goal = lambda goal: self.set_pose_goal(self.right_manipulator, goal)

        # Assign set update joint values function for each manipulator
        self.left_manipulator.set_joint_values = lambda goal: self.set_joint_values(self.left_manipulator, goal)
        self.right_manipulator.set_joint_values = lambda goal: self.set_joint_values(self.right_manipulator, goal)

        # Assign set value function for each gripper
        self.left_gripper.set_value = lambda value: self.set_gripper_value(self.left_gripper, value)
        self.right_gripper.set_value = lambda value: self.set_gripper_value(self.right_gripper, value)

        # Assign enable fingers collisions function for each gripper
        self.left_gripper.enable_fingers_collisions = lambda *value: self.enable_fingers_collisions('lgripper', *value)
        self.right_gripper.enable_fingers_collisions = lambda *value: self.enable_fingers_collisions('rgripper', *value)

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

    def enable_fingers_collisions(self, gripper_prefix, object_names, enable=True):
        # type: (str, list,  float) -> None
        """
        Disables or enables the collisions check between the fingers and a list of objects
        When closing the gripper, we should disable fingers collisions. If not, MoveIt! will
        fail its planning.
        :param gripper_prefix: either lgripper or rgripper
        :param object_names: a list of objects
        :param enable: set to True to enable / False to disable
        :return: None
        """
        while self.planning_scene_publisher.get_num_connections() < 1:
            rospy.loginfo("Waiting to subscribe to the /planning_scene")
            rospy.sleep(0.1)

        request = PlanningSceneComponents(
            components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        response = self.get_planning_scene(request)
        acm = response.scene.allowed_collision_matrix

        for name in object_names:
            if name not in acm.entry_names:
                # Add object to allowed collision matrix
                acm.entry_names += [name]
                for row in range(len(acm.entry_values)):
                    acm.entry_values[row].enabled += [False]
                new_row = deepcopy(acm.entry_values[0])
                acm.entry_values += {new_row}

        for index_entry_values, entry_values in enumerate(acm.entry_values):
            if gripper_prefix in acm.entry_names[index_entry_values]:
                for index_value, _ in enumerate(entry_values.enabled):
                    if acm.entry_names[index_value] in object_names:
                        if enable:
                            acm.entry_values[index_entry_values].enabled[index_value] = False
                        else:
                            acm.entry_values[index_entry_values].enabled[index_value] = True
            elif acm.entry_names[index_entry_values] in object_names:
                for index_value, _ in enumerate(entry_values.enabled):
                    if gripper_prefix in acm.entry_names[index_value]:
                        if enable:
                            acm.entry_values[index_entry_values].enabled[index_value] = False
                        else:
                            acm.entry_values[index_entry_values].enabled[index_value] = True
        planning_scene_diff = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
        self.planning_scene_publisher.publish(planning_scene_diff)
        rospy.sleep(1.0)

    def test(self):
        q = quaternion_from_euler(pi, 0, 0)
        april_4 = Pose(position=Point(*[0.6256, 0, 0.4]), orientation=Quaternion(*q))
        april_1 = Pose(position=Point(*[0.5124, -0.24, 0.4]), orientation=Quaternion(*q))
        april_0 = Pose(position=Point(*[0.5124, 0.24, 0.4]), orientation=Quaternion(*q))

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
    robot = XamyabRobot(visualize_trajectory=True)
    robot.test()
