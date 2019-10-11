#!/usr/bin/env python
"""
A generic perception class created to work with any camera.
It provides basic functionalities to retrieve raw data from
the camera (RGB, depth, and point cloud).
"""
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d # This is require for matplot lib
from cv_bridge import CvBridgeError, CvBridge

import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2


class GenericPerception(object):
    def __init__(self, depth, color, points, node_name='listener'):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        self.rgb_scaled_img = self.depth_scaled_img = self.point_cloud = None
        self.bridge = CvBridge()
        rospy.init_node(node_name, anonymous=True)
        rospy.Subscriber(depth, Image, self.depth_callback, queue_size=10)
        rospy.Subscriber(color, Image, self.color_callback, queue_size=10)
        rospy.Subscriber(points, PointCloud2, self.pointcloud_callback, queue_size=10)

    def color_callback(self, data):
        try:
            self.rgb_scaled_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

    def depth_callback(self, data):
        try:
            self.depth_scaled_img = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print e

    def pointcloud_callback(self, data):
        self.point_cloud = data

    def show_color(self):
        """
        Show color video
        :return: None
        """
        while not rospy.is_shutdown():
            rospy.Rate(10).sleep()
            if self.rgb_scaled_img is None:
                continue

            cv2.imshow('Color Image', self.rgb_scaled_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    def show_depth(self):
        """
        Show depth video
        :return: None
        """
        while not rospy.is_shutdown():
            rospy.Rate(10).sleep()
            if self.depth_scaled_img is None:
                continue

            d = cv2.applyColorMap(self.depth_scaled_img.astype(np.uint8), cv2.COLORMAP_RAINBOW)
            cv2.imshow('Depth Image', d)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    def get_pointcloud_coords(self):
        """
        Get the current point cloud coordinates
        :return: numpy array of 3D coordinates from the camera
        """
        while not rospy.is_shutdown():
            rospy.Rate(10).sleep()
            if self.point_cloud is not None:
                break

        coords = pc2.read_points(self.point_cloud, field_names=("x", "y", "z"))
        return list(coords)

    @staticmethod
    def plot_cube3d(coords):
        coords = np.asarray(coords)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(coords[:, 0], coords[:, 1], coords[:, 2], zdir='z', c='red')
        plt.axis('equal')
        plt.show()

    def run(self, run_test=False):
        if run_test:
            self.test()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def test(self):
        """
        Use this function to test your code
        :return: None
        """
        print "============ Show color"
        self.show_color()

        print "============ Show depth channel"
        self.show_depth()

        print "============ Print point cloud coordinates"
        coords = self.get_pointcloud_coords()
        print coords[0]

        print "============ Plot point cloud coords"
        GenericPerception.plot_cube3d(coords)


if __name__ == '__main__':
    # Change these to different rostopics according to the type of camera
    DepthSubscriber = '/kinect2/sd/image_depth_rect'
    ColorSubscriber = '/kinect2/sd/image_color'
    PointCloudSubscriber = '/kinect2/sd/points'

    perception = GenericPerception(DepthSubscriber, ColorSubscriber, PointCloudSubscriber)
    perception.run()
