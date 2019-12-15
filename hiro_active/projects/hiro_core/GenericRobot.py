#!/usr/bin/env python
"""
A generic class created to work with any robot controlled by the MoveIt! interface.
It has basic functions to retrieve robot information and to control all the
move groups belonging to the robot (by setting either the pose goal or joint values).
"""
import sys
from math import pi

import moveit_commander
import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene, DisplayTrajectory, RobotTrajectory
from moveit_msgs.srv import GetPlanningScene


class GenericRobot(object):

    def __init__(self, node_name='generic_robot_move_group', visualize_trajectory=True):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node(node_name, anonymous=True)
        self.visualize_trajectory = visualize_trajectory
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory,
                                                            queue_size=20)
        self.planning_scene_publisher = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10, latch=True)
        self.get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        rospy.sleep(0.5)

        # Find all available group names of the robot
        self.group_names = self.robot.get_group_names()
        self.move_groups = {}
        for name in self.group_names:
            self.move_groups[name] = moveit_commander.MoveGroupCommander(name)

    def contains(self, group_name):
        # type: (str) -> bool
        """
        Check whether the robot contains a particular move group
        :param group_name: move group name
        :return: bool: contains or not
        """
        if group_name in self.group_names:
            return True
        else:
            rospy.loginfo("Cannot find group:", group_name)
            return False

    def publish_trajectory(self, plan):
        # type: (RobotTrajectory) -> None
        """
        Publish trajectory so that we can visualize in Rviz
        :param plan:
        :return: None
        """
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.get_robot_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def plan_and_execute(self, move_group, joint_goal=None):
        # type: (str or moveit_commander.MoveGroupCommander, list) -> bool
        """
        Plan, display the trajectory and execute the plan
        :param move_group: MoveGroupCommander
        :param joint_goal: a list of joints
        :return: bool: is_success
        """
        is_success = False
        move_group = self.get_move_group_commander(move_group)
        move_group.set_start_state_to_current_state()

        # Planning
        plan = move_group.plan(joint_goal)

        # Visualizing and executing the trajectory
        while self.visualize_trajectory:
            rospy.loginfo("What do you want to do?"
                          "\n1. Re-visualize the plan (type 1)"
                          "\n2. Execute the plan (type execute)"
                          "\n3. Re-plan (type 3)"
                          "\n4. Do nothing (type 4)")
            rospy.sleep(0.5)
            user_input = raw_input("Input:").lower()
            if user_input == "1":
                self.publish_trajectory(plan)
                continue
            elif user_input == "execute":
                break
            elif user_input == "3":
                plan = move_group.plan(joint_goal)
                continue
            elif user_input == "4":
                return is_success
            else:
                rospy.logwarn("INVALID INPUT!")
                continue

        is_success = move_group.execute(plan, wait=True)

        # Retrying if failed
        while not is_success:
            rospy.logerr("Execution failed. Do you want to try again?"
                         "\n1. Yes"
                         "\n2. No")
            rospy.sleep(0.5)
            user_input = raw_input("Input:").lower()
            if user_input == "y" or user_input == "yes":
                is_success = self.plan_and_execute(move_group, joint_goal)
                break
            elif user_input == "n" or user_input == "no":
                break
            else:
                rospy.loginfo("Input invalid! Please type 'yes' or 'no'.")
                continue
        return is_success

    def set_pose_goal(self, move_group, pose_goal=None):
        # type: (str or moveit_commander.MoveGroupCommander, Pose) -> bool
        """
        Set pose goal for a particular move group
        :param move_group: move group name or commander
        :param pose_goal: the pose goal
        :return: bool: is successful
        """
        is_success = False
        move_group = self.get_move_group_commander(move_group)

        if pose_goal:
            move_group.set_pose_target(pose_goal)
            is_success = self.plan_and_execute(move_group)
            move_group.stop()
            move_group.clear_pose_targets()
        else:
            rospy.loginfo("Please provide a pose goal!")

        return is_success

    def set_joint_values(self, move_group, joint_goal):
        # type: (str or moveit_commander.MoveGroupCommander, list) -> bool
        """
        Set pose goal for a particular move group
        :param move_group: move group name or commander
        :param joint_goal: a list of the joint values
        :return: bool: is successful
        """
        is_success = False
        move_group = self.get_move_group_commander(move_group)

        current_joints = move_group.get_current_joint_values()
        if len(current_joints) != len(joint_goal):
            rospy.loginfo("Current joints (len=%i) are not the same as goal joints (len=%i)." \
                          % (len(current_joints), len(joint_goal)))
            return is_success

        is_success = self.plan_and_execute(move_group, joint_goal)
        move_group.stop()

        return is_success

    def _set_default_state(self, move_group, state):
        # type: (str or moveit_commander.MoveGroupCommander, str) -> bool
        """
        Set the default state of a particular move group
        :param move_group: move group name or commander
        :param state: the name of the default state defined in the SRDF file
        :return: bool: is successful
        """
        move_group = self.get_move_group_commander(move_group)
        move_group.set_named_target(state)
        return self.plan_and_execute(move_group)

    def get_move_group_commander(self, move_group):
        # type: (str or moveit_commander.MoveGroupCommander) -> moveit_commander.MoveGroupCommander
        if type(move_group) is str:
            if not self.contains(move_group):
                raise Exception("Invalid group name [%s]." % move_group)
            else:
                return self.move_groups[move_group]
        else:
            return move_group

    def get_robot_state(self):
        # type: () -> str
        return self.robot.get_current_state()

    def get_current_joint_values(self, group_name):
        # type: (str) -> list
        if self.contains(group_name):
            return self.move_groups[group_name].get_current_joint_values()

    def get_planning_frame(self, group_name):
        # type: (str) -> str
        if self.contains(group_name):
            return self.move_groups[group_name].get_planning_frame()

    def get_end_effector_link(self, group_name):
        # type: (str) -> str
        if self.contains(group_name):
            return self.move_groups[group_name].get_end_effector_link()


if __name__ == '__main__':
    robot = GenericRobot(visualize_trajectory=True)
    group_name = "left_manipulator"

    rospy.loginfo("============ rospy.loginfo(ing robot state")
    rospy.loginfo(robot.get_robot_state)

    rospy.loginfo("============ rospy.loginfo(ing joint values")
    rospy.loginfo(robot.get_current_joint_values(group_name))

    rospy.loginfo("============ Moving by updating joint states")
    joint_goal = robot.get_current_joint_values(group_name)
    joint_goal[2] += pi / 10
    rospy.loginfo(robot.set_joint_values(group_name, joint_goal))

    rospy.loginfo("============ Moving using pose goal")
    pose_goal = Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.3
    pose_goal.position.z = 0.5
    rospy.loginfo(robot.set_pose_goal(group_name, pose_goal))
