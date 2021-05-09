from math import pi

import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

from hiro_core.XamyabRobot import XamyabRobot, rospy
from shape_finder import geometric_center, where_to_grab
from collections import deque


class MovingShapes:
    def __init__(self):
        self.robot = XamyabRobot(visualize_trajectory=True)
        self.default_gripper_quaternion = Quaternion(*quaternion_from_euler(pi, 0, pi / 2))
        self.default_pos = Pose(position=Point(*[0.6256, -0.50, 0.2]), orientation=self.default_gripper_quaternion)
        self.object_size = 0.08 ### rand num rn (in meters) ###
        self.goal_positions = [Pose(position=Point(*[0.6256 + self.object_size - 0.02, self.object_size, 0.10]),
                                    orientation=self.default_gripper_quaternion),
                               Pose(position=Point(*[0.6256 - self.object_size + 0.02, self.object_size, 0.10]),
                                    orientation=self.default_gripper_quaternion),
                               Pose(position=Point(
                                   *[0.6256 + self.object_size - 0.02, self.object_size, 0.125 + self.object_size]),
                                   orientation=self.default_gripper_quaternion),
                               Pose(position=Point(
                                   *[0.6256 - self.object_size + 0.02, self.object_size, 0.125 + self.object_size]),
                                   orientation=self.default_gripper_quaternion)]

    def reset(self):
        self.robot.right_gripper.open()
        rospy.loginfo("Going home")
        self.robot.right_manipulator.home()

    def set_up_environment(self, object_name, transform_point):
        object_pose = PoseStamped()
        object_pose.header.frame_id = self.robot.right_manipulator.get_planning_frame()
        rospy.loginfo("Object {} position {}.".format(i, transform_point))
        object_pose.pose = Pose(position=Point(*transform_point))
        self.robot.right_gripper.enable_fingers_collisions(object_name, True)

        # Currently just adds boxes (can add spheres and cylinders)
        self.robot.scene.add_box(object_name, object_pose, (self.object_size, self.object_size, self.object_size))
        rospy.sleep(1)

    def move_to(self, pose_goal):
        pose_goal.position.z += 0.2
        pose_goal.orientation = self.default_gripper_quaternion
        # Only manipulate the right arm for now!
        self.robot.right_manipulator.set_pose_goal(pose_goal)

    def run(self, j):
        self.reset()
        rospy.loginfo("PROJECT READY")

        for i in range(j):
            object_name = "object" + str(i)
            transform_point = where_to_grab() ### function will get 2d array: (x,y) ###
            transform_point.append(0.05) # append z pos
            self.set_up_environment(object_name, transform_point)

            # Move to pos
            pose = Pose(position=Point(*transform_point))
            rospy.loginfo("Moving Object")
            self.move_to(pose)
            rospy.sleep(1)

            # Close gripper
            self.robot.right_manipulator.attach_object(object_name)
            rospy.sleep(1)
            rospy.loginfo("Closing gripper...")
            self.robot.right_gripper.enable_fingers_collisions([object_name], False)
            self.robot.right_gripper.close()

            # Move up a bit
            rospy.loginfo("Moving up a bit...")
            pose.position.z += self.object_size - 0.165
            self.move_to(pose)

            # Move to goal
            rospy.loginfo("Moving to goal...")
            self.move_to(self.goal_positions[i])

            # Open gripper
            rospy.loginfo("Opening gripper...")
            self.robot.right_gripper.open()
            rospy.sleep(1)
            self.robot.right_gripper.enable_fingers_collisions([object_name], True)
            self.robot.scene.remove_attached_object(self.robot.right_manipulator.get_end_effector_link(), name=object_name)

            # Move up a bit
            rospy.loginfo("Move up a bit...")
            self.goal_positions[i].position.z += self.object_size / 2 - 0.195
            self.move_to(self.goal_positions[i])
            rospy.sleep(1)


    def test(self):
        q = quaternion_from_euler(pi, 0, 0)
        shape_1 = Pose(position=Point(*[0.6256, 0, 0.4]), orientation=Quaternion(*q))
        # shape_2 = Pose(position=Point(*[0.5124, -0.24, 0.4]), orientation=Quaternion(*q))
        # shape_3 = Pose(position=Point(*[0.5124, 0.24, 0.4]), orientation=Quaternion(*q))

        self.robot.right_gripper.open()
        print("Going home")
        self.robot.right_manipulator.home()
        p1 = PoseStamped()
        p1.header.frame_id = self.robot.right_manipulator.get_planning_frame()
        p1.pose.position.x = 0.6256
        p1.pose.position.y = 0
        p1.pose.position.z = 0.05
        self.robot.scene.add_box("box", p1, (.08, .08, .08))

        rospy.sleep(1)
        self.robot.right_manipulator.set_pose_goal(shape_1)
        self.robot.right_manipulator.attach_object("box")
        rospy.sleep(1)
        print("Closing gripper")
        self.robot.right_gripper.enable_fingers_collisions(["box"], False)
        self.robot.right_gripper.set_value(50)
        self.robot.right_manipulator.home()
        self.robot.right_gripper.open()
        self.robot.right_gripper.enable_fingers_collisions(["box"], True)
        self.robot.scene.remove_attached_object(self.robot.right_manipulator.get_end_effector_link(), name="box")
        rospy.sleep(1)
        self.robot.scene.remove_world_object("box")
        rospy.sleep(1)

if __name__ == '__main__':
    project = MovingShapes()
    # project.test()
    project.run(1)
