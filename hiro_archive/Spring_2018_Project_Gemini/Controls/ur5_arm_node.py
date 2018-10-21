#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('irl')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from math import pi
from namedlist import namedlist
import urx

"""
Wrapper for UR5 robotic arm
Direct requests to the UR5 arm should be sent through this node

Arm can be used via joints(ROS) or via XYZ coordinates(TCP)

Note: if scripts or arm ever time out or freeze up, then just restart scripts and restart
the arm. It's just that there might be big load on the TCP/ROS connections.

Before this, the UR driver needs to be brought online:
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=10.42.0.175 or 10.42.0.54

then make sure to run this code with _robot_ip:=10.42.0.175 or 10.42.0.54
"""

Route = namedlist('Route', 'joints duration')

def rad2pi(joint_states):
    """control tablet reads in joint positions in degrees
    JointTrajectoryPoint is in radians. we can define as either, but this will convert to radians"""
    return [(j*pi/180) for j in joint_states]

class Arm():
    def __init__(self):
        #Setting up the connection to UR5 server
        rospy.init_node("arm_node", anonymous=True, disable_signals=True)
        self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        self.client.wait_for_server()
        print "Connected to server"

        self.JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(self.JOINT_NAMES):
                self.JOINT_NAMES[i] = prefix + name

        #HOME position of the arm
        self.HOME = [0,-90,0,-90,90,0]

        #Gesture dictionary, and building it
        self.gestures = {}
        self.build_gesture_dict()

        #Setup TCP
        tcp_ip = rospy.get_param("~robot_ip")
        self.coordinator = urx.Robot(tcp_ip)
        arm_dict = {'10.42.0.175':'_pollux','10.42.0.54':'_castor'}

        #Setting up subscriber
        self.joints_sub = rospy.Subscriber("behaviors_cmd%s"%arm_dict[tcp_ip], String, self.behaviors_callback)
        self.coordinates_sub = rospy.Subscriber("coordinates_cmd%s"%arm_dict[tcp_ip], String, self.coordinates_callback)
        self.status_pub = rospy.Publisher("arm_status%s"%arm_dict[tcp_ip], String, queue_size=0)
        self.query_sub = rospy.Subscriber("query_cmd%s"%arm_dict[tcp_ip], String, self.query_callback, queue_size=10)
        self.info_pub = rospy.Publisher("arm_info%s"%arm_dict[tcp_ip], String, queue_size=10)

        print "Running startup sequence"
        self.run_gesture("begin")
        print "Initialized, listening..."


    def coordinates_callback(self, data):
        """
        callback for the coordinates_cmd rostopic
        """

        coordinates = data.data
        pose = [float(item) for item in coordinates.split(' ')]
        print pose
        self.move_to_point(pose)

    def query_callback(self, data):
        if data.data == "joints":
            joints_msg = self.get_joints
            self.info_pub.publish(str(joints_msg))
        elif data.data == "coordinates":
            coord_msg = self.coordinator.getl()
            self.info_pub.publish(str(coord_msg))

    def move_to_point(self, pose):
        """
        move to a pose, a.k.a. a XYZ coordinate (with rotational vectors Rx Ry Rz)

        Note: move_to_point works only for small-medium scale movements. Because the joints
        can move from -360 to 360 (720 degree rotation), if the difference in moving is too far, it can
        interpret its movement as the +- 360 coordinate, which is equivalent. Therefore, XYZ movement should
        only be used when necessary and for relatively smaller movements (or break it into two smaller movements).
        """

        # sets status as busy and gives out current coordinates
        self.status_pub.publish("busy")
        old_pose = self.coordinator.getl()
        print "CURRENT COORDINATES: \n X: %3f Y: %3f Z: %3f Rx: %3f Ry: %3f Rz: %3f" %(old_pose[0],old_pose[1],old_pose[2],old_pose[3],old_pose[4],old_pose[5])

        # moves the robot according to length of input pose. if 6, then does a regular pose move. if 3, then
        # will move to new XYZ but use old Rx Ry Rz
        if len(pose) == 6:
            self.coordinator.movep(pose, acc=0.3, vel=1, wait=False)
        elif len(pose) == 3:
            self.coordinator.movep([pose[0], pose[1], pose[2], old_pose[3], old_pose[4], old_pose[5]], acc=0.3, vel=1, wait=False)
        else:
            print "INVALID COORDINATES, NOTHING HAPPENING"

        # service/client implementation to pause programming
        time.sleep(1)
        while self.coordinator.is_program_running():
            pass
        print "FINISHED MOVING"

        # confirm new coordinates
        new_pose = self.coordinator.getl()
        print "NOW AT: \n X: %3f Y: %3f Z: %3f Rx: %3f Ry: %3f Rz: %3f" %(new_pose[0],new_pose[1],new_pose[2],new_pose[3],new_pose[4],new_pose[5])

        self.status_pub.publish("free")


    def get_coordinates(self):
        """
        returns the current position of the arm in X, Y, Z, RX, RY, RZ
        """

        return self.coordinator.getl()


    def get_joints(self):
        """
        returns the current position of the arm in joint angles
        """

        return self.coordinator.getj()


    def behaviors_callback(self, data):
        """
        callback for behaviors_cmd rostopic
        """

        gesture = data.data
        print "GOT GESTURES: ", gesture
        self.run_gesture(gesture)


    def home_robot(self):
        """
        does a homing for running purposes, basically the home position but with the head turned around
        """

        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.JOINT_NAMES
        try:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position

            #Initialize trajectory with current position of robot
            g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]

            #Add HOME position
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=rad2pi(self.HOME), velocities=[0]*6, time_from_start=rospy.Duration(3)))

            #Send trajectory to arm
            print "Goal created, sending."
            self.client.send_goal(g)
            print "Waiting for result"
            self.client.wait_for_result()
            print "Gesture completed successfully"

        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise


    def build_gesture_dict(self):
        """
        Gesture dictionary for UR arm
        """

        ### SIMON SAYS GESTURES
        self.gestures["dance"] = [Route([83, -128, 40.75, 45,0,0], 2), Route([83, -38, -50, 176.4, 0,0],2)]
        self.gestures["touch_head"] = [Route([-85, -82, -40, -90, 90, 41], 2), Route([-85, -106, -117, -130, 90, 41], 3),
                                    Route([-85, -105, -117, -115, 90, 41], 0.5), Route([-85, -106, -117, -130, 90, 41], 0.5),
                                    Route([-85, -105, -117, -115, 90, 41], 0.5), Route([-85, -106, -117, -130, 90, 41], 0.5),
                                    Route([-85, -82, -40, -90, 90, 41], 2)]
        self.gestures["high5_self"] = [Route([-90, -84, -20.22, -89.94, 90, 52.43],2), Route([-90, -78.18, -118, -128.68, 89.97, 52.4], 4),
                                    Route([-90, -78, -125.85, -153, 89.98, 52.44], 0.5), Route([-90, -78.18, -118, -128.68, 89.97, 52.4], 0.5),
                                    Route([-90, -84, -20.22, -89.94, 90, 52.43],2)]
        self.gestures["dab"] = [Route([0,-65, -82, -43, 90, 41], 2), Route([-64, -96, -96, -42, 125, -38], 1.2),
                                Route([0,-65, -82, -43, 90, 41], 2)]
        self.gestures["starfish"] = [Route([0, -46, -129.13, -49.96, 91.68, 44.96], 3), Route([0,-90,0,-0.33,87.45,44.96], 1), Route(self.HOME, 2)]
        self.gestures["bow"] = [Route(self.HOME, 1.5), Route([0,-91, -98, -40, 88, 42], 3), Route(self.HOME, 2)]
        self.gestures["heart"] = [Route([-7.65,-73.45,-100,-18,91.69,37],2), Route([-49,-85,-77,-22,87,33],1),
                                    Route([-39.5,-96.7,-90,-28,127,42],1), Route([-12, -90, -124, -3, 92, 38.8],1),
                                    Route([38,-75.5,-124,12.36,58,37],1), Route([33.5, -75, -95.5, -15, 54, 26],1),
                                    Route([-7.65,-73.45,-100,-18,91.69,37],1)]
        self.gestures["wave"] = [Route([0,-65, -100, 18, 88, 42], 2),
                                Route([0, -74, -115, -37, 86, 42], 1), Route([0,-65, -100, 18, 88, 42], 1),
                                Route([0, -65, -111, -10, 88, 42], 1)]
        self.gestures["disco"] = [Route([-7.65,-73.45,-100,-18,91.69,37],2), Route([54.63, -104.26, -137.37, 17, 98, 51], 1.5),
                                Route([-1.62, -55.72, -125.98, -4.38, 90.59, 37], 1.5), Route([-63.46, -117, -17, 11, 88, 36], 1.5),
                                Route([-1.62, -55.72, -125.98, -4.38, 90.59, 37], 1.5), Route([54.63, -104.26, -137.37, 17, 98, 51], 1.5),
                                Route([-1.62, -55.72, -125.98, -4.38, 90.59, 37], 1.5), Route([-63.46, -117, -17, 11, 88, 36], 1.5),
                                Route([-1.62, -55.72, -125.98, -4.38, 90.59, 37], 1.5)]
        self.gestures["rub_tummy"] = [Route([-84, -84.9, -30, -81, 89.96, 47.99], 2), Route([-84.5, -75.6, -124, -134.8, 89.96, 47.99], 4),
                                Route([-84, -75.6, -124, -160, 89.96, 47.99], 0.5), Route([-84.5, -75.6, -124, -134.8, 89.96, 47.99], 0.5),
                                Route([-84, -75.6, -124, -160, 89.96, 47.99], 0.5),
                                Route([-84, -84.9, -30, -81, 89.96, 47.99], 2)]
        self.gestures["hug_self"] = [Route([0, -48, -123, -129, 82, 30], 4), Route([-19.3, -47, -122, -129, 82.3, 30], 1), Route([21.2, -47, -122, -111, 82.3, 30], 1),
                                    Route([-19.3, -47, -122, -129, 82.3, 30], 1), Route([21.2, -57, -100, -54, 68.9, 47], 2)]

        ### GENERAL GAME GESTURES
        self.gestures["begin"] = [Route([-4.14, -69.90, -64.94, -60.09, 86.98, 0], 2)]
        self.gestures["gloat"] = [Route([1.15, -83, -79.5, -33.88, 91.58, 40.72], 2), Route([1.15, -67, -104, -4.5, 91.5, 40.7], 0.7),
                                Route([1.15, -83, -79.5, -33.88, 91.58, 40.72], 0.7), Route([1.15, -67, -104, -4.5, 91.5, 40.7], 0.7)]
        self.gestures["done_game"] = self.gestures["wave"]
        self.gestures["get_set"] = [Route([0, -31.2, -119, -47, 93.6, 30], 3), Route([-32, -31, -131, -58, 122, 30], 1),
                                    Route([10, -36, -124, -34, 79, 30], 1), Route([-32, -31, -131, -58, 122, 30], 1),
                                    Route([0, -31.2, -119, -47, 93.6, 30], 2)]
        self.gestures["leader"] = [Route([0, -79.9, -87.5, -23.9, 90, 38.9], 2), Route([0, -74.3, -89.7, -128.7, 90, 43], 1),
                                Route([0, -74.3, -89.7, -165.2, 90, 43.1], 0.5), Route([0, -74.3, -89.7, -128.7, 90, 43], 0.5),
                                Route([0, -79.9, -87.5, -23.9, 90, 38.9], 1)]
        self.gestures["look"] = [Route([0, -62, -109, -28, 95, 32], 2)]
        self.gestures["sad"] = [Route([-91, -86.8, -40, -52, 90, 62], 2),
                                Route([-89.2, -78.4, -122.65, -83.4, 90, 62], 3), Route([-89.2, -78.4, -122.65, -113.5, 90, 62], 1),
                                Route([-89.3, -72, -105, -91.2, 90, 62], 3)]
        self.gestures["nod"] = [Route([-10.4, -77.81, -99.38, -17.65, 107.77, 37.25], 2), Route([-10.49, -77.81, -99.38, -61, 107.77, 37.25], 1),
                                Route([-10.4, -77.81, -99.38, -17.65, 107.77, 37.25], 1)]
        self.gestures["nod1"] = [Route([4.55, -99.1, -54.45, 23.07, 89.99, 52.43], 2), Route([4.55, -99.1, -54.45, -65, 90, 52.43], 1), Route([4.55, -99.1, -54.45, 23.07, 89.99, 52.43], 1)]

        ### CONNECT 4 GAME GESTURES
        self.gestures["c4_start"] = [Route([.20, -77.09, -5.45, -124.09, 83.41, .32], 2.5), Route([.20, -77.09, -5.45, -124.09, 96.29, 30], .7),
                                    Route([.20, -77.09, -5.45, -124.09, 70.16, -30], .7), Route([.20, -77.09, -5.45, -124.09, 83.41, .32], 1),
                                    Route([.21, -62.29, -115.70, -10.69, 85.97, 0], 2.5)]
        self.gestures["c4_home"] = [Route([99.73, -21.24, -161.23, -1.74, 78.85, 0], 5)]
        self.gestures["c4_end"] = [Route([0, -68.93, -102.56, -7.04, 86.35, 0], 1.5), Route([0, -68.93, -106.58, -14.47, 86.35, 0], .5),
                                    Route([0, -67.81, -98.65, .04, 86.35, 0], .5), Route([0, -68.93, -106.58, -14.47, 86.35, 0], .5),
                                    Route([0, -67.81, -98.65, .04, 86.35, 0], .5), Route([0, -90.76, -84.70, -4, 86.35, -20.33], .7)]
        self.gestures["c4_win"] = [Route([0, -60.02, -25.05, -94.00, 90, 0], 2), Route([0, -60.00, -20.05, -99.51, 90, 0], .5),
                                    Route([0, -60.02, -30.01, -88.44, 90, 0], .5), Route([0, -60.02, -20.05, -99.51, 90, 0], .5),
                                    Route([0, -60.02, -30.01, -88.44, 90, 0], .5), Route([0, -60.02, -20.05, -99.51, 90, 0], .5),
                                    Route([0, -60.02, -30.01, -88.44, 90, 0], .5), Route([0, -95, -5, -110, 90, 0], 1),
                                    Route([-15, -90, -5, -110, 90, -20], .7), Route([15, -90, -5, -110, 90, 20], .7),
                                    Route([-15, -90, -5, -110, 90, -20], .7), Route([15, -90, -5, -110, 90, 20], .7),
                                    Route([0, -90, -5, -110, 90, 0], 1), Route([0, -115, -95, 46, 90, 0], 2),
                                    Route([0, -115, -95, 46, 90, 15], 1),Route([0, -115, -95, 46, 90, 30], 1)]
        self.gestures["c4_loss"] = [Route([90, -70, -20, -125, 90, 0], 2), Route(self.HOME, 1.5), Route([0,-91, -98, -40, 88, 0], 3),
                                    Route(self.HOME, 2), Route([0, -106, -160, 60, 90, 0], 5),
                                    Route([-154, -103.74, -140, 38, 90, 0], 3), Route([-154, -103.74, -121.83, 64.54, 90, 0], 2),
                                    Route([-154, -109, -132.70, 27.76, 90, 0], 1), Route([0, -91, -160, 67.96, 90, 0], 3)]
        self.gestures["c4_load_token"] = [Route([120.55, -21.88, -139.94, -5.28, 96.65, 0], 3)]

        # column movements
        self.gestures["c4_move_1"] = [Route([95.87, -129.31, -5.41, -103.53, 94.48, 0], 5)]
        self.gestures["c4_move_2"] = [Route([92.63, -129.01, -5.41, -101.52, 94.14, 0], 5)]
        self.gestures["c4_move_3"] = [Route([89.27, -129.34, -5.16, -101.00, 94.13, 0], 5)]
        self.gestures["c4_move_4"] = [Route([85.98, -129.48, -5.43, -98.69, 92.92, 0], 5)]
        self.gestures["c4_move_5"] = [Route([85.50, -129.93, -5.43, -99.00, 84.06, 0], 5)]
        self.gestures["c4_move_6"] = [Route([84.96, -129.56, -5.42, -96.52, 75.68, 0], 5)]
        self.gestures["c4_move_7"] = [Route([81.08, -130.39, -5.39, -95.13, 76.69, 0], 5)]

        ### Tool Picker Game Gestures
        self.gestures["tp_welcome"] = [Route([-8.48, -59.81, -92.29, -27.75, 90.64, 1.46], 2)]
        self.gestures["tp_camera"] = [Route([20.02, -91.66, -80.65, -90.09, 90.64, 18.21],2)]

        ### Project Gemini Gestures
        self.gestures["pg_hover"] = [Route([90, -102.97, 77.89, -67.77, -90, 0],2)]
        self.gestures["pg_hover_alternate"] = [Route([90, -102.97, 77.89, -67.77, -90, 90],2)]
        self.gestures["pg_pickup_down_pollux"] = [Route([4.84, -96.21, 121.08, -115.34, -86.59, -84.39], 3)]
        self.gestures["pg_pickup_down_castor"] = [Route([146.27, -95.34, 120.23, -115.19, -92.42, 55.93], 3)]
        self.gestures["pg_pickup_up_pollux"] = [Route([4.52, -91.45, 61.04, -60.01, -86.63, -85.43], 3)]
        self.gestures["pg_pickup_up_castor"] = [Route([145.51, -91.14, 57.46, -56.61, -89.94, 55], 3)]

        ### IDLE GESTURES
        self.gestures["idle_stare_1"] = [Route([-40.20, -88.37, -23.93, -66.92, 76.21, 0], 2)]
        self.gestures["idle_stare_2"] = [Route([64.54, -86.64, -94.69, 8.90, 76.21, 0], 2)]
        self.gestures["idle_stare_3"] = [Route([0, -70, -29.26, -95.06, 93.53, 0], 2)]
        self.gestures["idle_sniff"] = [Route([37.5, -106.53, -139.53, 28.33, 84.01, 0], 2), Route([37.5, -106.53, -139.53, 29.81, 84.01, 0], .25),
                                    Route([37.5, -106.53, -139.53, 28.33, 84.01, 0], .25), Route([37.5, -106.53, -139.53, 29.81, 84.01, 0], .25),
                                    Route([37.5, -106.53, -139.53, 28.33, 84.01, 0], .25), Route([-43.25, -118.14, -113.51, 19.46, 84.01, 0], 2),
                                     Route([-43.25, -118.14, -113.51, 21.0, 84.01, 0], .25),  Route([-43.25, -118.14, -113.51, 19.46, 84.01, 0], .25),
                                      Route([-43.25, -118.14, -113.51, 21.0, 84.01, 0], .25),  Route([-43.25, -118.14, -113.51, 19.46, 84.01, 0], .25),
                                      Route([0, -66.28, -119.83, 6.66, 84.03, 0], 2)]
        self.gestures["idle_sneeze"] = [Route([0, -56.66, -76.3, 12.71, 87.62, 0], 5), Route([0, -70.64, -117.23, -12.97, 87.62, 0], 1),
                                    Route([0, -61.28, -94.58, -24.89, 87.62, 0], 2),]
        self.gestures["idle_butt_wiggle"] = [Route([0, -106, -154.52, 86.03, 87.62, 0], 2), Route([-5, -106, -154.52, 86.03, 92.85, 0], .3),
                                             Route([6, -106, -154.52, 86.03, 81.85, 0], .3),  Route([-5, -106, -154.52, 86.03, 92.85, 0], .3),
                                              Route([6, -106, -154.52, 86.03, 81.85, 0], .3), Route([-5, -106, -154.52, 86.03, 92.85, 0], .3),
                                               Route([6, -106, -154.52, 86.03, 81.85, 0], .3), Route([0, -106, -154.52, 86.03, 87.62, 0], 1)]
        self.gestures["idle_wander"] = [Route([-45, -91.4, -32.32, -45.60, 87.74, 0], 1.5), Route([45, -91.4, -32.32, -45.60, 87.74, 0], 5),
                                        Route([-45, -91.4, -32.32, -45.60, 87.74, 0], 5), Route([45, -91.4, -32.32, -45.60, 87.74, 0], 5)]
        self.gestures["idle_head_bobble"] = [Route([0, -78.11, -93.63, -10.47, 87.72, 0], 2), Route([0, -80, -99.63, -24.88, 87.72, 0], .5),
                                            Route([0, -76.11, -87.33, 6, 87.72, 0], .7), Route([0, -80, -99.63, -24.88, 87.72, 0], .5),
                                            Route([0, -76.11, -87.33, 6, 87.72, 0], .7), Route([0, -78.11, -93.63, -10.47, 87.72, 0], 2)]
        self.gestures["idle_curiosity"] = [Route([-90, -66.40, -90.69, -35.79, 90, 0], 5), Route([-90, -58.20, -58.50, -81.49, 90, 0], .7),
                                            Route([-90, -58.20, -58.50, -81.49, 90, -45], 1.5)]
        self.gestures["idle_sleep"] = [Route([5, -37.48, -160.66, -64.52, 138.11, -46.81], 5), Route([-10, -37.48, -160.66, -64.52, 138.11, -46.81], 2),
                                        Route([20, -37.48, -160.66, -64.52, 138.11, -46.81], 3), Route([-10, -37.48, -160.66, -64.52, 138.11, -46.81], 4),
                                        Route([20, -37.48, -160.66, -64.52, 138.11, -46.81], 4.5), Route([5, -37.48, -160.66, -64.52, 138.11, -46.81], 4)]


    def run_gesture_incremental(self, gesture):
        """
        Runs a gesture based on what it finds in the dictionary
        Increments, so requires user input to move from keypose to keypose
        """

        gest2run = self.gestures[gesture]
        if type(gest2run[-1]) == int:
            repeat_num = gest2run[-1]
            gest2run.pop(-1)
        else:
            repeat_num = 1
        g = FollowJointTrajectoryGoal()
        try:
            #Add each gesture from the dictionary to the trajectory
            for gest in gest2run:
                if type(gest) == str:
                    cmd = gest.split(":")[0]
                    val = gest.split(":")[1]

                    if cmd == "SLEEP":
                        time.sleep(float(val))
                    continue

                g = FollowJointTrajectoryGoal()
                cur_joint_states = rospy.wait_for_message("joint_states", JointState)
                g.trajectory = JointTrajectory()
                g.trajectory.joint_names = self.JOINT_NAMES

                #Initialize trajectory with current position of robot
                g.trajectory.points = [JointTrajectoryPoint(positions=cur_joint_states.position, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
                g.trajectory.joint_names = self.JOINT_NAMES

                assert len(gest.joints) == 6
                g.trajectory.points.append(JointTrajectoryPoint(positions=rad2pi(gest.joints), velocities=[0]*6, time_from_start=rospy.Duration(gest.duration)))

                #Send trajectory to arm
                print "Goal created, sending."
                self.client.send_goal(g)
                print "Waiting for result"
                self.client.wait_for_result()
                # print "Gesture completed, do next move?"
                inp = raw_input("Ready for next move?")

                if (inp == "n"):
                    raise KeyboardInterrupt

        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise

        except AssertionError:
            print "RUNNING: ", gest.joints
            print "WRONG LENGTH OF JOINTS: GOT %d, EXPECTED 6" % (len(gest.joints))
            raise


    def run_gesture(self, gesture):
        """
        Runs a gesture based on what it finds in the dictionary
        """

        gest2run = self.gestures.get(gesture, None)

        if gest2run is None:
            print "Gesture: ", gesture, " is not recognized"
            return

        self.status_pub.publish("busy")

        if type(gest2run[-1]) == int:
            repeat_num = gest2run[-1]
            gest2run.pop(-1)
        else:
            repeat_num = 1

        g = FollowJointTrajectoryGoal()
        joint_states = rospy.wait_for_message("joint_states", JointState)
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.JOINT_NAMES
        try:
            joints_pos = joint_states.position

            #Initialize trajectory with current position of robot
            g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
            duration = 0

            for repeat in range(repeat_num):
                #Add each gesture from the dictionary to the trajectory
                for gest in gest2run:
                    duration += gest.duration
                    if len(gest.joints) == 6:
                        g.trajectory.points.append(JointTrajectoryPoint(positions=rad2pi(gest.joints), velocities=[0]*6, time_from_start=rospy.Duration(duration)))
                    else:
                        print gest.joints
                        print "WRONG LENGTH: GOT %d, EXPECTED 6" % (len(gest.joints))
                        raise TypeError

            #Send trajectory to arm
            print "Goal created, sending."
            self.client.send_goal(g)
            print "Waiting for result"
            self.client.wait_for_result()
            print "Gesture completed successfully"

        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

        self.status_pub.publish("free")


    def test_run(self):
        """
        runs a bunch of different behaviors (and coordinates), used for testing
        """

        try:
            inp = raw_input("Ready to run? y/n: ")[0]
            if (inp == 'y'):
                # self.run_gesture("bow")
                # self.run_gesture("starfish")
                # self.run_gesture("disco")
                # self.run_gesture("heart")
                # self.run_gesture("wave")
                # self.run_gesture("dab")
                self.run_gesture("idle_butt_wiggle")
                # self.run_gesture("c4_home")
                # self.run_gesture("c4_move_3")
                # self.run_gesture("c4_move_1")

                # self.move_to_point([.015, -.461, .434, .195, 3.26, -0.056])
                # time.sleep(10)
            else:
                print "Halting program"
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise


    def run(self):
        print "UR5 control is running"
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                r.sleep()
            except KeyboardInterrupt:
                print "\nCLOSING TCP"
                self.coordinator.close()
                print "TCP CLOSED"
                break


if __name__ == '__main__':
    a = Arm()
    a.run()
    # a.test_run()
    # a.coordinator.close()
    # a.home_robot()
    # a.coordinator.close()
