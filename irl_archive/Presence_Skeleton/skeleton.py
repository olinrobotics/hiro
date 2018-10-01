#!/usr/bin/env python

"""
To run skeleton.py, please run in terminals:

roscore
roslaunch skeleton_markers markers_from_tf.launch
roslaunch openni_launch openni.launch

roscd skeleton_markers
rosrun rviz rviz -d markers_from_tf.rviz

and then this script
"""

import sys
import rospkg
rospack = rospkg.RosPack()
PACKAGE_PATH = rospack.get_path("edwin")
sys.path.append(PACKAGE_PATH + '/sight')
sys.path.append(PACKAGE_PATH + '/motion')

import rospy
import math
import numpy as np
from std_msgs.msg import String, Int16
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from edwin.msg import Bones, HHH
import time
import tf
import cv2
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError

"""
This class constructs and processes a skeleton of the user that can track 15 body parts
in 3D space. It then pushes these body points to various topics for usage elsewhere too.

Note that depending on where the user stands the legs may not be accurately tracked, and the
tracking is in meters from the origin, which is the Kinect.
"""


class Skeleton(object):
    """
    Class to hold and use skeleton of users
    """
    def __init__(self):
        #init ROS node
        rospy.init_node("skeleton", anonymous=True)

        #image stuff
        self.bridge = CvBridge()
        self.cv_image = None

        #skeleton stuff
        self.body_points = None
        self.head = (0,0,0)
        self.torso = (0,0,0)
        self.Rhand = (0,0,0)
        self.Lhand = (0,0,0)

        #running
        self.running = True
        self.detected = False


        #subscribing to skeleton markers
        rospy.Subscriber("/skeleton_markers", Marker, self.constructSkeleton, queue_size=10)

        #subscribing to Kinect camera
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.renderImage, queue_size=10)

        #makes two publishers, one for the head and hands, and one for the entire body
        self.presencePub = rospy.Publisher("/presence", HHH, queue_size=10)
        self.skelePub = rospy.Publisher("/skeleton", Bones, queue_size=10)



    def constructSkeleton(self, skeleton):
        """
        Parses out skeleton_markers data

        Body points is a list of 15 markers that denotes body parts, as per the skeleton
        They are in this order in the list:

        0 - head
        1 - neck
        2 - torso

        3 - right shoulder
        4 - right elbow
        5 - right hand

        6 - right hip
        7 - right knee
        8 - right foot

        9 - left shoulder
        10 - left elbow
        11- left hand

        12 - left hip
        13 - left knee
        14 - left foot
        """
        #gets the skeleton and extracts the head and hands
        if len(skeleton.points) == 15:
            self.body_points = skeleton.points
            raw_head = self.body_points[0]
            raw_torso = self.body_points[2]
            raw_Rhand = self.body_points[5]
            raw_Lhand = self.body_points[11]



            #converts the head and hands to Kinect coordinate frame
            self.head = self.transform_skel2kinect(raw_head)
            self.torso = self.transform_skel2kinect(raw_torso)
            self.Rhand = self.transform_skel2kinect(raw_Rhand)
            self.Lhand = self.transform_skel2kinect(raw_Lhand)

            self.detected = True





    def renderImage(self, image):
        """
        Renders an image using opencv
        """
        #extracts the image
        self.cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")

        #converts head coordinates to image coordinates for visualization
        imagex = 320 - int(self.head[0])
        imagey = 240 - int(self.head[1])
        cv2.circle(self.cv_image, (imagex, imagey), 10, 255, thickness = -1)

        #converts hand coordinates to image coordinates for visualization
        imagex = 320 - int(self.Rhand[0])
        imagey = 240 - int(self.Rhand[1])
        cv2.circle(self.cv_image, (imagex, imagey), 10, 255, thickness = -1)
        imagex = 320 - int(self.Lhand[0])
        imagey = 240 - int(self.Lhand[1])
        cv2.circle(self.cv_image, (imagex, imagey), 10, 255, thickness = -1)



    def transform_skel2kinect(self, bodypart):
        """
        makes a transformation from the coordinates of the skeleton to the coordinates
        of the Kinect image
        """
        #converts based on triangular ratio from a camera FOV
        x = bodypart.y/bodypart.x * 640
        y = bodypart.z/bodypart.x * 480
        z = bodypart.x
        return (x, y, z)

    def send_out_data(self):
        """
        publishes out data to both presence detection topic and simon says topic
        """

        #make the messages
        bones = Bones()
        presence = HHH()

        #fill them with data
        if self.body_points is not None:
            bones.h = self.body_points[0]
            bones.n = self.body_points[1]
            bones.t = self.body_points[2]
            bones.rs = self.body_points[3]
            bones.re = self.body_points[4]
            bones.rh = self.body_points[5]
            bones.rp = self.body_points[6]
            bones.rk = self.body_points[7]
            bones.rf = self.body_points[8]
            bones.ls = self.body_points[9]
            bones.le = self.body_points[10]
            bones.lh = self.body_points[11]
            bones.lp = self.body_points[12]
            bones.lk = self.body_points[13]
            bones.lf = self.body_points[14]

            self.skelePub.publish(bones)

        if not self.detected:
            self.head = (0,0,0)

        presence.headx = int(self.head[0])
        presence.heady = int(self.head[1])
        presence.headz = int(self.head[2]*1000)
        presence.torsox = int(self.torso[0])
        presence.torsoy = int(self.torso[1])
        presence.torsoz = int(self.torso[2])
        presence.rhandx = int(self.Rhand[0])
        presence.rhandy = int(self.Rhand[1])
        presence.rhandz = int(self.Rhand[2])
        presence.lhandx = int(self.Lhand[0])
        presence.lhandy = int(self.Lhand[1])
        presence.lhandz = int(self.Lhand[2])

        # print self.head[0], self.head[1], self.head[2]

        self.presencePub.publish(presence)
        self.detected = False


    def run(self):
        """
        main run function for edwin
        """
        print "running presence skeleton"
        r = rospy.Rate(10)
        time.sleep(2)

        cv2.namedWindow("chicken")

        while not rospy.is_shutdown():
            if self.cv_image is not None:
                cv2.imshow("chicken", self.cv_image)
                cv2.waitKey(3)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            self.send_out_data()

            r.sleep()

if __name__ == "__main__":
    skull = Skeleton()
    skull.run()
