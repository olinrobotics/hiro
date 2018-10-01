#!/usr/bin/env python
from __future__ import absolute_import
import rospy
import rospkg
import random
import math
import time
import numpy as np
from std_msgs.msg import String, Int16
import rospkg
import pickle
from irl.srv import *
import os, sys
from io import open

class RouteCreator(object):
    def __init__(self, default_init):
        rospy.init_node('route_creator', anonymous=True)
        # self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=2)
        self.debug_pub = rospy.Publisher('/arm_debug', String, queue_size=1)
        self.arm_status = rospy.Publisher('/route_arm_status', String, queue_size=10)
        rospy.Subscriber('/arm_debug', String, self.create_callback, queue_size=2)
        time.sleep(1)

        rospack = rospkg.RosPack()
        self.PACKAGE_PATH = rospack.get_path("irl")

        self.route_dictionary = {}
        self.create_route_dictionary()

        self.initialized = default_init
        print "Initializing"

    def create_route_dictionary(self):
        ##Single set routes

        ###################               General actions           #####################

        self.route_dictionary["R_stare"] = "R_stare; 3296, 2308, 999, 0, 0, 0"
        self.route_dictionary["R_ttt"] = "R_ttt; 200, 2400, 1800, 720, 240, 21"
        self.route_dictionary["R_look"] = "R_look; 2564, 1974, 3013, 110, 225, 0"
        self.route_dictionary["R_impat"] = "R_impat; 0, 3621, 4860, 545, 120, 21"
        self.route_dictionary["R_laugh"] = "R_laugh; 1000, 700, 7000, -456, 150, 21, 2000, 2000, 2000, 580, 90, 21, 1500, 1500, 3500, 338, 90, 21, 1700, 1700, 3200, 338, 90, 21, 1500, 1500, 3500, 338, 90, 21, 1700, 1700, 3200, 338, 90, 21, 1500, 1500, 3500, 338, 90, 21, 1700, 1700, 3200, 338, 90, 21,  997, 997, 6490, 306, 0, 21, 844, 1134, 6495, 306, 390, 21, 1131, 847, 6497, 305, -300, 21, 844, 1134, 6495, 306, 390, 21, 1131, 847, 6497, 305, -300, 21, 997, 997, 6490, 306, 0, 21"
        self.route_dictionary["R_curious"] = "R_curious; 3664, 1774, 3013, 0, 0, 0"
        self.route_dictionary["R_nudge"] = "R_nudge; 2500, 1900, 1700, 630, 270, 0, 3000, 2400, 2200, 57, 240, 0, 2700, 2100, 1700, 569, 270, 0, 3200, 2600, 2200, 22, 211, 0"
        self.route_dictionary["R_sad_turn"] = "R_sad_turn; 0, 2900, 200, 930, 270, 21"
        self.route_dictionary["R_inhale"] = "R_inhale; 1000, -2700, 1000, 547, 300, 21"
        self.route_dictionary["R_sigh_up"] = "R_sigh_up; 1000, -3000, 1700, 209, 180, 21"
        self.route_dictionary["R_sigh_down"] = "R_sigh_down; 1000, -2500, 0, 940, 165, 21"
        self.route_dictionary["R_curl_up"] = "R_curl_up; 1573, 1574, 1262, 69, 240, 21"
        self.route_dictionary["R_stare_yonder"] = "R_stare_yonder; 1000, -2500, 6000, 80, 240, 21"
        self.route_dictionary["R_sniff"] = "R_sniff; 1000, 4000, -700, 358, 240, 21, 4000, -100, -500, 346, 240, 21, 3600, 1770, -200, 346, 240, 21, 2564, 1974, 3013, 110, 225, 0"
        self.route_dictionary["R_sneeze"] = "R_sneeze; 1100, 1100, 7000, -850, 210, 21, 2400, 2400, 1000, 602, 210, 21, 2400, 2400, 1000, 151, 240, 21, 2564, 1974, 3013, 110, 225, 0"
        self.route_dictionary["R_scrunch_up"] = "R_scrunch_up; 400, 3500, 300, 186, 240, 21"
        self.route_dictionary["R_head_bobble"] = "R_head_bobble; 3600, -100, 3500, 187, 240, 21, 4000, -100, 5000, 310, 240, 21, 4000, -100, 2000, 117, 240, 21, 4000, -100, 5000, 310, 240, 21, 4000, -100, 2000, 117, 240, 21, 3600, -100, 3500, 187, 240, 21, 2564, 1974, 3013, 110, 225, 0"
        self.route_dictionary["R_squirrel"] = "R_squirrel; 4500, -1200, 5000, -195, 240, 21, 3000, 4000, 3000, 82, 240, 21, 2564, 1974, 3013, 110, 225, 0"
        self.route_dictionary["R_spin_position"] = "R_spin_position; 1500, 1500, 800, 64, 240, 21"
        self.route_dictionary["R_greet1"] = "R_greet1; 3665, 1774, 3013, 0, 0, 0"
        self.route_dictionary["R_leaving"] = "R_leaving; -2689, 2612, 375, 27, 0, 18"
        self.route_dictionary["R_wakeup"] = "R_wakeup; 0, 3523, 5032, 1, 0, 0"
        self.route_dictionary["R_playful"] = "R_playful; 2027, 981, 98, -11, 0, 72"
        self.route_dictionary["R_sleep"] = "R_sleep; 1450, 542, 1400, 645, -150, 0"
        self.route_dictionary["R_look_at_draco"] = "R_look_at_draco; 0, 3000, 3000, 228, 238, 0, 0, 999, 6000, 302, 210, 0"
        self.route_dictionary["R_stare_3"] = "R_stare_3; 3500, 200, 4500, 108, 225, 0"

        ###################               Simon Says actions           #####################

        self.route_dictionary['R_starfish'] = "R_starfish; 1497, 998, 2002, 509, 150, 0, 117, 82, 7497, -877, 210, 0"
        self.route_dictionary["R_bow"] = "R_bow; 1697, 1199, 4999, -374, 210, 0, 3698, 2999, 0, 655, 150, 0"
        self.route_dictionary["R_rub_tummy"] = "R_rub_tummy; 245, 2439, 390, 1151, 240, 21, 275, 2774, 610, 1208, 240, 21, 245, 2439, 390, 1151, 240, 21, 275, 2774, 610, 1208, 240, 21, 245, 2439, 390, 1151, 240, 21, 275, 2774, 610, 1208, 240, 21, 245, 2439, 390, 1151, 240, 21, 2564, 1974, 3013, 110, 225, 0"
        self.route_dictionary["R_disco"] = "R_disco; 1000, 4000, 0, 566, 210, 0, 2000, 0, 2500, 284, 150, 0, 1500, -4000, 5000, -207, 210, 0, 2000, 0, 2500, 284, 150, 0, 1000, 4000, 0, 566, 210, 0, 2000, 0, 2500, 284, 150, 0, 1500, -4000, 5000, -207, 210, 0, 2000, 0, 2500, 284, 150, 0, 1000, 4000, 0, 566, 210, 0, 2564, 1974, 3013, 110, 225, 0"
        self.route_dictionary["R_dab"] = "R_dab; 2564, 1974, 3013, 110, 225, 0, 947, -948, 1501, 532, -270, 0,  2564, 1974, 3013, 110, 225, 0"
        self.route_dictionary["R_heart"] = "R_heart; 3500, 2500, 0, 400, 150, 0, 4500, 1500, 3000, 74, 210, 0, 4000, 2000, 4000, -200, 240, 0, 3500, 2500, 3000, 407, 150, 0, 3000, 3000, 4000, -343, 210, 0, 2500, 3500, 3000, 107, 180, 0, 3500, 2500, 0, 400, 150, 0, 2564, 1974, 3013, 110, 225, 0"
        self.route_dictionary["R_hug_self"] = "R_hug_self; 1388, 1389, 1456, 794, 510, 0, 1694, 994, 1456, 794, 510, 0, 1043, 1664, 1456, 794, 510, 0, 1694, 994, 1456, 794, 510, 0, 1043, 1664, 1456, 794, 510, 0, 1694, 994, 1456, 794, 510, 0, 1043, 1664, 1456, 794, 510, 0, 2564, 1974, 3013, 110, 225, 0"
        self.route_dictionary["R_wave"] = "R_wave; 0, 1000, 6999, -923, 150, 0, 0, 148, 7320, -1363, 150, 0, 0, 1605, 6604, -303, 150, 0, 0, 148, 7320, -1363, 150, 0, 0, 1605, 6604, -303, 150, 0, 0, 148, 7320, -1363, 150, 0, 0, 1605, 6604, -303, 150, 0, 2564, 1974, 3013, 110, 225, 0"
        self.route_dictionary["R_touch_head"] = "R_touch_head; 0, 4000, -1700, 1586, 240, 0, 0, 4198, -2200, 1534, 150, 0, 0, 4000, -1700, 1586, 240, 0, 0, 4198, -2200, 1534, 150, 0, 0, 4000, -1700, 1586, 240, 0, 0, 4198, -2200, 1534, 150, 0, 0, 4000, -1700, 1586, 240, 0, 2564, 1974, 3013, 110, 225, 0"
        self.route_dictionary["R_high5_self"] = "R_high5_self; 0, 3500, 2000, 1326, 210, 0, 0, 3000, 1700, 1455, 225, 0, 0, 3500, 2000, 1326, 210, 0, 2564, 1974, 3013, 110, 225, 0"

        self.route_dictionary['R_get_set'] = "R_get_set; 3498, 1999, 1999, -32, 180, 0"
        self.route_dictionary['R_done_game'] = "R_done_game; 2488, 1917, 4809, 281, 150, 0, 2099, 1617, 2651, 600, 150, 0"
        self.route_dictionary['R_leader'] = "R_leader; 1205, 843, 6635, 285, 180, 0"
        self.route_dictionary['R_praise'] = "R_praise; 3200, 2300, 2800, 444, 210, 0, 3200, 2300, 3600, -277, 180, 0, 3200, 2300, 2800, 444, 210, 0, 3200, 2300, 3600, -277, 180, 0, 3200, 2300, 2800, 444, 210, 0, 3200, 2300, 3600, -277, 180, 0, 3600, 2700, 3200, 76, 390, 0"
        self.route_dictionary['R_sad'] = "R_sad; -2499, 772, 700, 748, 150, 0, -2600, 772, 1500, 685, 150, 0, -2400, 772, 0, 853, -60, 0, -4558, 1467, 2271, 796, 150, 0"


        ###################               Set Game actions           #####################
        self.route_dictionary['R_set_center'] = "R_set_center; 0, 3500, 2000, 847, 180, 0"

        ###################               Sudoku actions               #####################
        self.route_dictionary['R_sudoku_center'] = "R_sudoku_center; 0, 3400, 4700, 856, 50, 0"

        ##Routes with lists
        self.route_dictionary["R_1_lookaround"] = ["R_1_lookaround; 4000, 1500, 3000, 185, 240, 21", "R_2_lookaround; 500, 4000, 2000, 185, 240, 21", "R_3_lookaround; 3000, 2000, 4000, -39, 240, 21"]
        self.route_dictionary["R_1_weep"] = ["R_1_weep; 1000, -2700, 400, 739, 150, 21", "R_2_weep; 1000, -2700, 300, 819, 150, 21", "R_3_weep; 1000, -2700, 200, 870, 150, 21", "R_4_weep; 1000, -2700, 100, 950, 120, 21", "R_5_weep; 1000, -2700, 0, 1030, 120, 21"]

        pickle.dump(list(self.route_dictionary.keys()), open(self.PACKAGE_PATH + '/params/routes.txt', 'wb'))


    def request_cmd(self, cmd):
        rospy.wait_for_service('arm_cmd', timeout=15)
        cmd_fnc = rospy.ServiceProxy('arm_cmd', arm_cmd)
        print "I have requested the command"

        try:
            resp1 = cmd_fnc(cmd)
            print "command done"


        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def setup_initial_routes(self):
        inital_routes = ["R_ttt", "R_laugh", "R_nudge", "R_look", "R_sad_turn", "R_get_set", "R_sudoku_center", "R_set_center"]

        for r in inital_routes:
            msg = "create_route:: " + self.route_dictionary[r]
            print "Sending message: ", msg
            self.request_cmd(msg)
            time.sleep(.5)

        self.debug_pub.publish("ROUTE CREATE DONE")
        time.sleep(1)

    def create_callback(self, cmd_raw):
        # self.create_route_dictionary()
        if "HOMING DONE" in cmd_raw.data:
            self.setup_initial_routes()
            self.initialized = True
            return

        if self.initialized == False:
            return

        if "RUN FAILED" in cmd_raw.data:
            missed_route = cmd_raw.data.split(" ")[1]
        elif "NOT DEFINED" in cmd_raw.data:
            missed_route = cmd_raw.data.split(" ")[1][:-1]
        else:
            return

        print "MISSED ROUTE: ", missed_route
        route = self.route_dictionary.get(missed_route, None)
        self.arm_status.publish('busy')

        if route == None:
            print "Route: " + missed_route + " not found"
            return
        elif type(route) == list:
            for r in route:
                msg = "create_route:: " + r
                print "Sending message: ", msg
                self.request_cmd(msg)
        elif type(route) == str:
            msg = "create_route:: " + route
            print "Sending message: ", msg
            self.request_cmd(msg)

        time.sleep(1)
        self.request_cmd("run_route:: " + route.split(";")[0])

        self.arm_status.publish("free")

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    rc = RouteCreator(True)
    rc.setup_initial_routes()
    rc.run()
