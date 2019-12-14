import numpy as np
import math
import rospy
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image, PointCloud2
from apriltag_ros.msg import AprilTagDetectionArray
from numpy.linalg import norm
import statistics
import sys


assert sys.argv[1]

class Distances:
    def __init__(self):
        self.bridge = CvBridge()
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('listener', anonymous=True)
        #rospy.Subscriber(depth, Image, self.depth_callback, queue_size=10)
        #rospy.Subscriber('/kinect2/hd/image_color', Image, self.color_callback, queue_size=10)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.Distance_callback, queue_size=10)
        #self.rgb_scaled_img =
        self.data =  None

    def Distance_callback(self,data):
        self.data = data

    def apriltag_coord(self):


        tag_pos = [[], [], [], [],[]]

        tags=[]
        for i in range(0,100):
            while not rospy.is_shutdown():
                rospy.Rate(10).sleep()
                if self.data is None:
                    continue

                for tag in self.data.detections:
                    tag_id = tag.id[0]
                    tag_pos[tag_id].append(np.asarray([tag.pose.pose.pose.position.x,tag.pose.pose.pose.position.y,tag.pose.pose.pose.position.z,1]))

                break
        for i in tag_pos:
            tags.append(np.mean(i,0))
        return tags, tag_pos[4]

    def calib(self):
        two = np.asarray([625.6, 0, 184, 1])
        one = np.asarray([512.4, -240, 0, 1])
        zero = np.asarray([512.4, 240, 0, 1])
        three = np.asarray([625.6, 240, 92, 1])
        coords, tag1 = self.apriltag_coord()
        # Want to find this

        calib = np.dot(np.linalg.inv([zero, one, two, three]), np.asarray([coords[0], coords[1],coords[2],coords[3]]))
        print(calib)

        np.savetxt(sys.argv[1],calib)

    def testing(self):
        coords, _ = self.apriltag_coord()
        calib = np.loadtxt(sys.argv[1])
        # print(calib)
        # print(coords)
        test = np.dot(coords[2],np.linalg.inv(calib))
        print(test)
    def run(self):
        # Get calibration matrix
        # self.calib()

        self.testing()


if __name__ == '__main__':

    distance = Distances()
    distance.run()
