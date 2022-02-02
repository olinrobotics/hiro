# !/usr/bin/env python
from math import pi

import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from apriltag_ros.msg import AprilTagDetectionArray

from hiro_core.XamyabRobot import XamyabRobot, rospy
from collections import deque


class StackingCubes:
    def __init__(self):
        self.robot = XamyabRobot(visualize_trajectory=True)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.distance_callback, queue_size=10)
        self.bridge = CvBridge()
        self.cube_size = 0.092
        self.default_gripper_quaternion = Quaternion(*quaternion_from_euler(pi, 0, pi / 2))
        self.default_pos = Pose(position=Point(*[0.6256, -0.50, 0.2]), orientation=self.default_gripper_quaternion)
        self.goal_positions = [Pose(position=Point(*[0.6256 + self.cube_size - 0.02, self.cube_size, 0.10]),
                                    orientation=self.default_gripper_quaternion),
                               Pose(position=Point(*[0.6256 - self.cube_size + 0.02, self.cube_size, 0.10]),
                                    orientation=self.default_gripper_quaternion),
                               Pose(position=Point(
                                   *[0.6256 + self.cube_size - 0.02, self.cube_size, 0.125 + self.cube_size]),
                                   orientation=self.default_gripper_quaternion),
                               Pose(position=Point(
                                   *[0.6256 - self.cube_size + 0.02, self.cube_size, 0.125 + self.cube_size]),
                                   orientation=self.default_gripper_quaternion)]
        self.data = None
        self.april_tags = [deque(maxlen=30), deque(maxlen=30), deque(maxlen=30), deque(maxlen=30)]
        self.project_ready = False
        self.transform_matrix = np.linalg.inv(np.load("Calibration_Matrix.npy"))

    def distance_callback(self, data):
        self.data = data
        if self.data is not None:
            for tag in self.data.detections:
                if tag.id[0] <= 4:
                    self.april_tags[tag.id[0] - 1].append(np.asarray(
                        [tag.pose.pose.pose.position.x, tag.pose.pose.pose.position.y,
                         tag.pose.pose.pose.position.z - self.cube_size / 2 - 0.025,
                         1]))
            self.project_ready = True
            for tag in self.april_tags:
                if len(tag) < 10:
                    self.project_ready = False

    def set_up_environment(self):
        for i in range(1, 5):
            cube_name = "cube" + str(i)
            cube_pose = PoseStamped()
            cube_pose.header.frame_id = self.robot.right_manipulator.get_planning_frame()
            transform_point = self.transform(np.mean(self.april_tags[i - 1], 0))[:3] / 10
            rospy.loginfo("Cube {} position {}.".format(i, transform_point))
            cube_pose.pose = Pose(position=Point(*transform_point))
            self.robot.scene.add_box(cube_name, cube_pose, (self.cube_size, self.cube_size, self.cube_size))
            rospy.sleep(1)

    def move_tip(self, pose_goal):
        pose_goal.position.z += 0.2
        pose_goal.orientation = self.default_gripper_quaternion
        # Only manipulate the right arm for now!
        self.robot.right_manipulator.set_pose_goal(pose_goal)

    def pick_cube(self, id):
        cube_name = "cube" + str(id)
        transform_point = self.transform(np.mean(self.april_tags[id - 1], 0))[:3] / 10
        rospy.loginfo("Cube {} position {}.".format(id, transform_point))
        pose = Pose(position=Point(*transform_point))

        # Move to cube
        rospy.loginfo("Moving to Cube {}...".format(id))
        self.move_tip(pose)
        rospy.sleep(1)

        # Close gripper
        self.robot.right_manipulator.attach_object(cube_name)
        rospy.sleep(1)
        rospy.loginfo("Closing gripper...")
        self.robot.right_gripper.enable_fingers_collisions([cube_name], False)
        self.robot.right_gripper.close()

        # Move up a bit
        rospy.loginfo("Moving up a bit...")
        pose.position.z += self.cube_size - 0.165
        self.move_tip(pose)

        # Move to goal
        rospy.loginfo("Moving to goal...")
        self.move_tip(self.goal_positions[id - 1])

        # Open gripper
        rospy.loginfo("Opening gripper...")
        self.robot.right_gripper.open()
        rospy.sleep(1)
        self.robot.right_gripper.enable_fingers_collisions([cube_name], True)
        self.robot.scene.remove_attached_object(self.robot.right_manipulator.get_end_effector_link(), name=cube_name)

        # Move up a bite
        rospy.loginfo("Move up a bit...")
        self.goal_positions[id - 1].position.z += self.cube_size / 2 - 0.195
        self.move_tip(self.goal_positions[id - 1])
        rospy.sleep(1)

    def run(self):
        while not self.project_ready:
            rospy.sleep(.1)
            continue
        # Enable collisions
        self.robot.right_gripper.enable_fingers_collisions(["cube1", "cube2", "cube3", "cube4"], True)

        rospy.loginfo("PROJECT READY")
        self.set_up_environment()
        self.pick_cube(id=1)
        self.pick_cube(id=2)
        self.pick_cube(id=3)
        self.pick_cube(id=4)

    def reset(self):
        self.robot.right_gripper.open()
        self.move_tip(self.default_pos)

    def apriltag_coord(self, iter=10000):
        tags = {2: [], 3: [], 6: [], 7: []}

        for i in range(iter):
            while not rospy.is_shutdown():
                rospy.sleep(0.01)
                if self.data is None:
                    continue

                for tag in self.data.detections:
                    tag_id = tag.id[0]
                    if tag_id in tags:
                        tags[tag_id].append(np.asarray(
                            [tag.pose.pose.pose.position.x, tag.pose.pose.pose.position.y,
                             tag.pose.pose.pose.position.z,
                             1]))

                break
        for i in tags:
            tags[i] = np.mean(tags[i], 0)
        return tags

    def calibrate(self):
        seven = np.asarray([5.124, -2.40, 0, 1])
        six = np.asarray([5.124, 2.40, 0, 1])
        two = np.asarray([6.256, 0, 1.84, 1])
        three = np.asarray([6.256, 2.40, .92, 1])
        coords = self.apriltag_coord()

        expected = np.asarray([seven, six, two, three])
        coords = np.asarray([coords[7], coords[6], coords[2], coords[3]])
        calib = np.dot(np.linalg.inv(expected), coords)
        np.save("Calibration_Matrix_1", calib)

        # Test calibration
        coords = self.apriltag_coord(iter=20)
        coords = np.asarray([coords[7], coords[6], coords[2], coords[3]])
        calib = np.load("Calibration_Matrix_1.npy")
        test = np.dot(coords, np.linalg.inv(calib))

        # MSE
        mse = np.mean((coords - expected) ** 2, 0)

        print("Coords", test)
        print("MSE", mse)

    def transform(self, coord):
        transformed_coords = np.dot(coord, self.transform_matrix)
        return transformed_coords

    def test(self):
        q = quaternion_from_euler(pi, 0.01, 0.01)
        april_4 = Pose(position=Point(*[0.6256, 0, 0.4]), orientation=Quaternion(*q))
        april_1 = Pose(position=Point(*[0.5124, -0.24, 0.4]), orientation=Quaternion(*q))
        april_0 = Pose(position=Point(*[0.5124, 0.24, 0.4]), orientation=Quaternion(*q))
        april_4 = Pose(position=Point(*[0.6256, 0, 0.4 - 0.15]), orientation=Quaternion(*q))

        self.robot.right_gripper.open()
        print "Homing"
        self.robot.right_manipulator.home()
        p = PoseStamped()
        p.header.frame_id = self.robot.right_manipulator.get_planning_frame()
        p.pose.position.x = 0.6256
        p.pose.position.y = 0
        p.pose.position.z = 0.05

        self.robot.scene.add_box("box", p, (.08, .08, .08))
        rospy.sleep(1)
        self.robot.right_manipulator.set_pose_goal(april_4)
        self.robot.right_manipulator.attach_object("box")
        rospy.sleep(1)
        print "Closing gripper"
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
    project = StackingCubes()
    # project.calibrate()
    # april_4 = Pose(position=Point(*[0.6256, 0, 0.3]), orientation=project.default_gripper_quaternion)
    # project.robot.right_manipulator.set_pose_goal(april_4)
    # project.reset()
    project.run()
