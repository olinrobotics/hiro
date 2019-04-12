import numpy as np
import random
import cv2
import rospy
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import numpy as np

class Perception:
    def __init__(self, depth, color, points):
            self.bridge = CvBridge()
            # In ROS, nodes are uniquely named. If two nodes with the same
            # name are launched, the previous one is kicked off. The
            # anonymous=True flag means that rospy will choose a unique
            # name for our 'listener' node so that multiple listeners can
            # run simultaneously.
            rospy.init_node('listener', anonymous=True)
            rospy.Subscriber(depth, Image, self.depth_callback, queue_size=10)
            rospy.Subscriber(color, Image, self.color_callback, queue_size=10)
            rospy.Subscriber(points, PointCloud2, self.pointcloud_callback, queue_size=10)
            self.rgb_scaled_img = self.depth_scaled_img = self.point_cloud = None


    def color_callback(self,data):
        try:
            self.rgb_scaled_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

    def depth_callback(self,data):
        try:
            self.depth_scaled_img  = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)

    def pointcloud_callback(self,data):
        self.point_cloud = data

    def show_color(self):
        while not rospy.is_shutdown():
            rospy.Rate(10).sleep()
            if self.rgb_scaled_img is None:
                continue

            c = cv2.applyColorMap(self.rgb_scaled_img.astype(np.uint8), cv2.COLORMAP_RAINBOW)
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

        while self.point_cloud is None:
            print "No point cloud found"

        self.gen = pc2.read_points(self.point_cloud, field_names=("x", "y", "z"))

        coords = []
        count = 0
        for i, p in enumerate(gen):
            coords.append(p)
            count += 1
            if count > 100000:
                break

        return np.asarray(coords)

    def plot_cube3d(coords):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(coords[:, 0], coords[:, 1], coords[:, 2], zdir='z', c='red')
        plt.axis('equal')
        plt.show()

    def run(self):
        #show color
        self.show_color()

        #show depth
        self.show_depth()

        #collect pointcloud coords
        self.get_pointcloud_coords()

        #plot pointcloud coords
        #self.plot_cube3d(coords)

        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()

if __name__ == '__main__':
    #Change these to different rostopics according to the type of camera
    DepthSubscriber = '/kinect2/sd/image_depth_rect'
    ColorSubscriber = '/kinect2/hd/image_color'
    PointCloudSubscriber = '/kinect2/hd/points'

    perception = Perception(DepthSubscriber,ColorSubscriber,PointCloudSubscriber)
    perception.run()
