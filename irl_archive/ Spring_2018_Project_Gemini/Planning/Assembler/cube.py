#!/usr/bin/env python
"""
A basic Digital Cube implementation for convenience in working with python

Has the exact same functionality as that of the Cube ros msg data structure, and
is meant to be used as such

The idea is to convert a ros msg Cube into a Digital_Cube inside python scripts,
and then re-convert into ros msg Cubes when sending along ros topics.

For the sake of not having to repeatedly make Digital_Cubes, in addition to holding
attributes functionality also exists to set and reset cube attributes.

This script is meant to be imported as package into digital_env.py and assembly_instructor.py
"""
import rospy
import numpy as np
from std_msgs.msg import String, Int16
import time



class Digital_Cube(object):
    """
    The Digital_Cube class, which holds attributes matching that of a ros msg Cube,
    and also additional functionality for assembling and creating grid environments
    """

    def __init__(self):
        self.height = 0
        self.connections = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.activated = False

    def turn_on(self, height, x, y, z):
        """
        sets a cube's attributes and "turns it on" in a grid environment, as we
        try not to destroy and make new cubes every time.
        """

        self.activated = True
        self.height = height
        self.x = x
        self.y = y
        self.z = z

    def set_connectivity(self, connectivity):
        """
        sets the connectivity of the cube, used in sorting into instructions
        """

        self.connections = connectivity


    def turn_off(self):
        """
        resets Cube so it can be used again
        """
        self.activated = False
        self.connections = 0
        self.height = 0
        self.x = 0
        self.y = 0
        self.z = 0
