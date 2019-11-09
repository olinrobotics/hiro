#!/usr/bin/env python
"""
A generic class created to work with any robot controlled by the MoveIt! interface.
It has basic functions to retrieve robot information and to control all the
move groups belonging to the robot (by setting either the pose goal or joint values).
"""
import sys

import moveit_commander
import rospy
import geometry_msgs.msg
import moveit_msgs.msg
from math import pi


class GenericRobot(object):

    def __init__(self, node_name='generic_robot_move_group', visualize_trajectory=True):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node(node_name, anonymous=True)
        self.visualize_trajectory = visualize_trajectory
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
        rospy.sleep(0.2)

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
            print "Cannot find group:", group_name
            return False

    def publish_trajectory(self, plan):
        # type: (moveit_msgs.msg.RobotTrajectory) -> None
        """
        Publish trajectory so that we can visualize in Rviz
        :param plan:
        :return:
        """
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.get_robot_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def plan_and_execute(self, move_group, joint_goal=None):
        # type: (moveit_commander.MoveGroupCommander, list) -> bool
        """
        Plan, display the trajectory and execute the plan
        :param move_group: MoveGroupCommander
        :param joint_goal: a list of joints
        :return: bool: is_success
        """
        plan = move_group.plan(joint_goal)
        while True:
            if self.visualize_trajectory:
                print "What do you want to do?" \
                      "\n1. Re-visualize the plan (type 1)" \
                      "\n2. Execute the plan(type execute)" \
                      "\n3. Re-plan (type 3)" \
                      "\n4. Do nothing (type 4)"
                user_input = raw_input("Input:")
                if user_input == "1":
                    self.publish_trajectory(plan)
                    continue
                elif user_input == "execute":
                    break
                elif user_input == "3":
                    plan = move_group.plan(joint_goal)
                    continue
                else:
                    return False
            else:
                break

        is_success = move_group.execute(plan, wait=True)
        return is_success

    def set_pose_goal(self, group_name, pose_goal=None):
        # type: (str, geometry_msgs.msg.Pose) -> bool
        """
        Set pose goal for a particular move group
        :param group_name: move group name
        :param pose_goal: the pose goal
        :return: bool: is successful
        """
        is_success = False
        if pose_goal:
            if self.contains(group_name):
                move_group = self.move_groups[group_name]
                move_group.set_pose_target(pose_goal)
                is_success = self.plan_and_execute(move_group)
                move_group.stop()
                move_group.clear_pose_targets()
        else:
            print "Please provide a pose goal!"

        return is_success

    def update_joint_values(self, group_name, joint_goal):
        # type: (str, list) -> bool
        """
        Set pose goal for a particular move group
        :param group_name: move group name
        :param joint_goal: the joint goal
        :return: bool: is successful
        """
        is_success = False
        if self.contains(group_name):
            current_joints = robot.get_current_joint_values(move_group)
            if len(current_joints) != len(joint_goal):
                print "Current joints are not the same as goal joints."
                return is_success

            is_success = self.plan_and_execute(self.move_groups[group_name])
            self.move_groups[group_name].stop()

        return is_success

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
    robot = GenericRobot()
    move_group = "left_manipulator"

    print "============ Printing robot state"
    print robot.get_robot_state

    print "============ Printing joint values"
    print robot.get_current_joint_values(move_group)

    print "============ Moving by updating joint states"
    joint_goal = robot.get_current_joint_values(move_group)
    joint_goal[2] += pi / 10
    print robot.update_joint_values(move_group, joint_goal)

    print "============ Moving using pose goal"
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.3
    pose_goal.position.z = 0.5
    print robot.set_pose_goal(move_group, pose_goal)
