#!/usr/bin/env python
"""
To run presence_detection.py, please run in terminal:

roscore
roslaunch openni_launch openni.launch
rosrun edwin edwin_bodies

roslaunch skeleton_markers markers_from_tf.launch
roscd skeleton_markers
rosrun rviz rviz markers_from_tf.rviz
rosrun edwin skeleton.py
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
from edwin.msg import *
import time
import tf
from edwin.srv import arm_cmd

"""
Presence Detection can find and track the nearest user to Edwin. It can also detect
hand waves have Edwin physically follow the user around the room.

Update March 2017

This code has also been augmented to follow specifically the user's head rather
than the center of mass of their body, which allows for a more accurate and realistic
interaction. The parameters and code logic was also improved for smoother and cleaner
execution.

"""

class Coordinates:
    """
    helper class to keep track of each individual person's coordinates and status
    """
    def __init__(self, ID, x, y, z):
        self.ID = ID
        self.X = x
        self.Y = y
        self.Z = z
        self.acknowledged = False


    def set_Coordinates(self, x, y, z):
        self.X = x
        self.Y = y
        self.Z = z


    def change_Presence(self):
        self.acknowledged = not self.acknowledged


class Presence:
    """
    main class for detecting presence, following people, and waving
    """
    def __init__(self):
        rospy.init_node('edwin_presence', anonymous = True)

        # person of interest
        self.person = None

        #coordinates of the person edwin's interacting with
        self.coordx = 0
        self.coordy = 0
        self.coordz = 0

        #edwin's own coordinates
        self.edwinx = 0
        self.edwiny = 0
        self.edwinz = 0

        #looping
        self.running = True

        #found or naw
        self.detected = False

        #tracking the user's information
        self.head = None
        self.prev_head = None

        # status
        self.status = None

        # #subscribing to edwin_bodies, from Kinect
        # rospy.Subscriber('body', SceneAnalysis, self.presence_callback, queue_size=10)
        #
        # #subscribing to edwin_wave, from Kinect
        # rospy.Subscriber('wave_at_me', Int16, self.wave_callback, queue_size=10)

        #subsrcibing to st.py's arm_debug, from Edwin
        rospy.Subscriber('arm_debug', String, self.edwin_location, queue_size=10)

        #subscribing to the skeleton topic, from Skeleton Markers, to get the head
        rospy.Subscriber('presence', HHH, self.get_HHH, queue_size=10)

        #subscribing to the arm status for service calls
        rospy.Subscriber('/arm_status', String, self.status_callback, queue_size=10)

        #subscribing to the brain for commands
        rospy.Subscriber('/presence_cmd', String, self.presence_callback, queue_size=10)


        #setting up ROS publishers to Edwin commands
        self.behavior_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
        self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=1)
        self.idle_pub = rospy.Publisher('idle_cmd', String, queue_size=10)
        self.detection_pub = rospy.Publisher('detected', String, queue_size=10)


        # tf transformations between Kinect and Edwin
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        time.sleep(2)
        print "Starting presence_detection.py"


    def request_cmd(self, cmd):
        rospy.wait_for_service('arm_cmd', timeout=15)
        cmd_fnc = rospy.ServiceProxy('arm_cmd', arm_cmd)
        print "I have requested the command"

        try:
            resp1 = cmd_fnc(cmd)
            print "command done"


        except rospy.ServiceException, e:
            print "Service call failed: %s"%e



    def presence_callback(self, data):
        """
        receives commands from brain
        """

        command = data.data
        if command == "start":
            self.running = True
        elif command == "stop":
            self.running = False


    def status_callback(self, data):
        """
        receives current movement status of arm
        """

        print "arm status callback", data.data
        if data.data == "busy" or data.data == "error":
            self.status = 0
        elif data.data == "free":
            self.status = 1

    def check_completion(self):
        """
        makes sure that actions run in order by waiting for response from service
        """

        time.sleep(3)
        while self.status == 0:
            pass


    def get_HHH(self, msg):
        """
        subscribes to skeleton to get and parse the current user's head
        """
        self.prev_head = self.head
        self.head = msg.headx, msg.heady, msg.headz
        # self.torso = msg.torsox, msg.torsoy, msg.torsoz
        # self.Rhand = msg.Rhandx, msg.Rhandy, msg.Rhandz
        # self.Lhand = msg.Lhandx, msg.Lhandy, msg.Lhandz

        # if the distance to Kinect is 0, then that means it's not seeing anything
        if self.head[2] == 0:
            self.person = None
        else:
            xpos, ypos, zpos = self.kinect_transform(self.head[0], self.head[1], self.head[2])

            #either sets up a new presence or updates a person's presence
            if self.person is None:
                self.person = Coordinates(1, xpos, ypos, zpos)
            else:
                self.person.set_Coordinates(xpos, ypos, zpos)





    def edwin_location(self, res):
        """
        subscribes to arm_debug to get and parse edwin's current location
        """
        if res.data[0:5] == "WHERE":
            #gets edwin's location
            where = res.data[5:]

            #massive string formatting - takes the string, splits by a formatter, then takes the array index that holds the XYZ,
            #then strips that string and splits it by spacing, and then takes the XYZ
            #reason for this massive formatting is b/c sent data format is not consistent
            where = where.split("\r\n")[2].strip().split('  ')

            #gets rid of empty strings that result from formatting
            where = filter(None, where)[0:3]

            #makes everything numbers that can be used as coordinates for Edwin
            where = [int(float(coord) * 10) for coord in where]

            self.edwinx = where[0]
            self.edwiny = where[1]
            self.edwinz = where[2]


    def reset(self):
        """
        resets some parameters of presence such that it can continuously be rerun
        """

        # person of interest
        self.person = None

        #coordinates of the person edwin's interacting with
        self.coordx = 0
        self.coordy = 0
        self.coordz = 0

        #edwin's own coordinates
        self.edwinx = 0
        self.edwiny = 0
        self.edwinz = 0

        #looping
        self.running = True

        #found or naw
        self.detected = False

        #tracking the user's information
        self.head = None
        self.torso = None
        self.Rhand = None
        self.Lhand = None

        # status
        self.status = None

    def find_new_people(self):
        """
        greets people if they are newly tracked presence,
        responds to waves
        """
        #greets people, only greets once while they're in the camera's view and are center of attention


        if (self.person is not None) and (self.person.acknowledged == False):
            self.person.acknowledged = True
            print "I see you!"
            self.idle_pub.publish("idle:stop")
            time.sleep(2)

            greeting = ["R_nudge","R_look"]
            for msg in greeting:
                    self.behavior_pub.publish(msg)
                    self.check_completion()


            self.detection_pub.publish('found')

        elif self.person is None:
            print "I don't see you"
            self.detection_pub.publish('nothing')


    def follow_people(self):
        """
        follows the nearest person's body around
        """
        #finds the person of interest's coordinates and then converts them to Edwin coordinates
        if (self.person is not None) and (self.person.acknowledged == True):
            trans = self.kinect_to_edwin_transform([self.person.X, self.person.Y, self.person.Z])
            if trans is not None:
                xcoord, ycoord, zcoord = self.edwin_transform(trans)

                #the person's coordinates are updated here, edwin's coordinates are updated in the callback
                self.coordx = xcoord
                self.coordy = ycoord
                self.coordz = zcoord

                #after coordinates are calculated, checks if the person has moved enough to respond, and then responds
                if abs(self.coordx - self.edwinx) > 400 or abs(self.coordy - self.edwiny) > 400 or abs(self.coordz - self.edwinz) > 400:
                    msg = "move_to:: " + str(self.coordx) + ", " + str(self.coordy) + ", " + str(self.coordz) + ", " + str(11)
                    self.request_cmd(msg)


    def attention(self):
        """
        DEPRECATED NOT NEEDED FOR SKELETON
        finds the nearest person and specifically targets them
        """
        center_of_attention = 0
        distance = 10000
        for person in self.peoples:
            if person is not None:
                if person.X < distance: #person's depth is now their X position in edwin frame
                    center_of_attention = person.ID
                    distance = person.X

        if center_of_attention != 0:
            return center_of_attention


    def kinect_to_edwin_transform(self, person):
        """
        transforms coordinates from the kinect reference frame to edwin's reference frame
        kinect reference frame = center of Kinect camera
        edwin reference frame = center of edwin' base, the plateau below his butt
        """
        self.br.sendTransform((person[0], person[1], person[2]),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            "human",
                            "kinect")

        try:
            (trans,rot) = self.listener.lookupTransform('/world', '/human', rospy.Time(0))
            return trans

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None


    def kinect_transform(self, x, y, z):
        """
        transforms coordinates such that the kinect coordinates are centered
        on the camera rather than to an arbitrary corner
        """
        xposition = x
        yposition = y
        zposition = z

        return zposition, xposition, yposition


    def edwin_transform(self, coordinates):
        """
        additional transform of coordinates that have been changed from kinect to
        edwin to make sure that edwin can appropriately move to said coordinates
        """
        edwinx = int(3.459 * coordinates[0] - 1805)
        edwiny = int(6.167 * coordinates[1] - 2400)
        edwinz = int(37.50 * coordinates[2] + 2667)

        if edwinx > 4000:
            edwinx = 4000
        elif edwinx < 0:
            edwinx = 0

        if edwiny > 4000:
            edwiny = 4000
        elif edwiny < -400:
            edwiny = -400

        if edwinz > 3500:
            edwinz = 3500
        elif edwinz < -600:
            edwinz = -600

        return edwinx, edwiny, edwinz


    def run(self):
        """
        main run function for edwin
        """
        print "running presence detection"
        r = rospy.Rate(10)
        time.sleep(2)

        while not rospy.is_shutdown():
            if self.running:
                self.find_new_people()
                self.follow_people()

            r.sleep()

if __name__ == "__main__":
    detector = Presence()
    detector.run()
