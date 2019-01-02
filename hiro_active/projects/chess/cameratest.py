import numpy as np
import random
import cv2
import rospy
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool

class CameraType:
    MID_ROW_D400 = 238
    MID_COL_D400 = 315
    DEPTH_UNIT_D400 = 1
    OFFSET_D400 = 0

    MID_ROW_SR300 = 218
    MID_COL_SR300 = 292
    DEPTH_UNIT_SR300 = 0.124986647279
    OFFSET_SR300 = 0.005

    def __init__(self, camera_type="D400", width=640, height=480):
        if camera_type is "D400":
            self.MID_ROW = CameraType.MID_ROW_D400
            self.MID_COL = CameraType.MID_COL_D400
            self.DEPTH_UNIT = CameraType.DEPTH_UNIT_D400
            self.OFFSET = CameraType.OFFSET_D400
        else:
            self.MID_ROW = CameraType.MID_ROW_SR300
            self.MID_COL = CameraType.MID_COL_SR300
            self.DEPTH_UNIT = CameraType.DEPTH_UNIT_SR300
            self.OFFSET = CameraType.OFFSET_SR300
        self.IMAGE_WIDTH = width
        self.IMAGE_HEIGHT = height


class Perception:
    def __init__(self, camera_type="D400", width=640, height=480):
        self.bridge = CvBridge()
        rospy.init_node('depth_cam', anonymous=True)
        rospy.Subscriber('/camera/color/image_raw', Image, self._rgb_callback, queue_size=10)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self._depth_callback, queue_size=10)
        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self._pointcloud_callback, queue_size=10)
        self.cam = CameraType(camera_type, width, height)
        self.rgb_data = self.depth_data = self.point_cloud = None
       

    def _rgb_callback(self, data):
        try:
            self.rgb_data = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

    def _depth_callback(self, data):
        try:
            self.depth_data = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print e

    def _pointcloud_callback(self, data):
        self.point_cloud = data

    def show_rgb(self):
        """
        Show RGB video
        :return: None
        """
        while self.rgb_data is None:
            pass

        while not rospy.is_shutdown():
            rospy.Rate(10).sleep()
            cv2.imshow('RGB Image', self.rgb_data)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    def show_depth(self):
        """
        Show depth video
        :return: None
        """
        while self.depth_data is None:
            pass

        while not rospy.is_shutdown():
            rospy.Rate(10).sleep()
            d = self.depth_data * self.cam.DEPTH_UNIT
            d = cv2.applyColorMap(d.astype(np.uint8), cv2.COLORMAP_RAINBOW)
            cv2.imshow('Depth Image', d)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    def show_rgbd(self):
        """
        Show both rgb and depth video
        :return: None
        """
        while self.depth_data is None:
            pass

        while not rospy.is_shutdown():
            rospy.Rate(10).sleep()
            d = self.depth_data * self.cam.DEPTH_UNIT
            d = cv2.applyColorMap(d.astype(np.uint8), cv2.COLORMAP_RAINBOW)
            dc = np.concatenate((d, self.rgb_data), axis=1)
            cv2.imshow('RGB & Depth Image', dc)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()


if __name__ == '__main__':
    print "Perception running"
    perception = Perception()
    perception.show_rgb()
