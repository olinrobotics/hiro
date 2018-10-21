#!/usr/bin/env python

"""
The Digital Environment (testing RL, just 3x3 one layer, instead of 3x3x3)

by Kevin Zhang

Creates random structures using building blocks to be sent to other modules for testing

This script is meant to be imported as a package in env_RL.py

remember to have catkin_make for the ros msgs

how it works:
1. resets its Environment
2. builds a new environment, randomly choosing where to build in a 3x3x3 space, and
only building on top of previously built cubes (so no floating cubes, builds in layers)
3. with the built environment, extracts information and makes usable Cube structs
4. sends out the info and waits for another prompt
"""

import rospy
import numpy as np
from std_msgs.msg import String, Int16
import time
from irl.msg import Grid_Cube, Grid_Structure
from cube import Digital_Cube
from assembly_instructor import Assembler
import itertools


class Environment(object):
    """
    the Environment class, the main class that holds the digitial environment and
    its creation capabilities
    """

    def __init__(self):
        self.vCube = np.vectorize(Digital_Cube) # faster python struct using numpy
        self.env_size = 3 # dimension of env
        self.env_height = 1

        self.env = np.empty((self.env_size,self.env_size, self.env_height), dtype=object) # the digital environment
        # filling out the environment with cubes, but they start deactivated
        for x, y, z in itertools.product(*map(xrange,(self.env_size, self.env_size, self.env_height))):
            self.env[x,y,z] = self.vCube()
        self.cubes = [] # the final list of cube information
        self.build_prob = 0.7 # the randomizer
        self.asm = Assembler()


    def reset(self):
        """
        resets the environment for another round of creation,
        deactivates all cubes in the environment
        resets cube list and build_prob
        """

        for x, y, z in itertools.product(*map(xrange,(self.env_size, self.env_size, self.env_height))):
            self.env[x,y,z].turn_off()
        self.cubes = []
        self.build_prob = 0.7


    def build_struct(self):
        """
        first builds a structure in the environment, goes up in layers and activates
        cubes randomly to "build the structure", only activating on top of previously
        activated cubes for a sound structure

        then creates Cubes and fills them in with information about the activated cubes in
        the environment, this becomes the self.cubes list
        """

        # first building the environemnt digitally, going up in layers and activating cubes
        layer = 0
        while layer < self.env_height:
            # self.build_prob -= 0.1
            for y, x in itertools.product(*map(xrange,(self.env_size, self.env_size))):
                if  layer == 0 or self.env[x,y,layer-1].activated:
                    prob = np.random.random_sample()
                    if prob < self.build_prob:
                        self.env[x,y,layer].turn_on(layer+1, x, y, layer)
            layer += 1

        # then making actual usable cubes from the environment and filling out all the information
        for y, x, z in itertools.product(*map(xrange,(self.env_size, self.env_size, self.env_height))):
            if self.env[x,y,z].activated:
                connections = 0
                for c in [[x+1, y],[x-1, y], [x,y+1], [x,y-1]]:
                    if all(n >= 0 and n < self.env_size for n in c) and self.env[c[0],c[1],z].activated:
                        connections += 1
                self.env[x,y,z].set_connectivity(connections)
                self.cubes.append(self.make_real_cube(self.env[x,y,z]))

        # ensures that the environment is never empty
        if self.cubes == []:
            self.reset()
            self.build_struct()


    def make_real_cube(self, cube):
        """
        converts between python class cube and ros data structure cube
        """

        real_cube = Grid_Cube()
        real_cube.height = cube.height
        real_cube.connections = cube.connections
        real_cube.x = cube.x
        real_cube.y = cube.y
        real_cube.z = cube.z
        return real_cube


    def sequence_struct(self):
        """
        shuffles the struct for better testing, and then sends the struct out
        for use by whatever other module
        """

        np.random.shuffle(self.cubes)

        # makes a ros structure to hold all the cubes, useful for publishing
        struct = Grid_Structure()
        for block in self.cubes:
            struct.building.append(block)

        # print "BUILDING BLOCKS MADE, NOW SEQUENCING"

        self.asm.set_cube_list(struct)
        return self.asm.sequence()




    def print_struct(self):
        """
        for testing and convenience, prints out the newly made structure, using
        birds eye view and column heights representation
        """

        # creates the birds eye view representation
        column = 0
        printed_map = np.zeros((self.env_size,self.env_size))
        total = 0
        for y, x in itertools.product(*map(xrange, (self.env_size, self.env_size))):
                k = 0
                while k < self.env_height and self.env[x,y,k].activated:
                    k += 1
                    total += 1
                printed_map[x,y] = k

        # then prints it
        print "STRUCT AS SEEN FROM BIRD EYE VIEW"
        print total, "total cubes in this structure\n"
        for  i in range(self.env_size):
            print ' | '.join(map(str,map(int, printed_map[:,i])))
            if i < self.env_size-1:
                print "---------"
        print "\n"
        return total


    def create_a_struct(self):
        """
        callback that prompts the script to refresh and make a brand new random
        struct for testing
        """

        self.reset()
        self.build_struct()
        # self.print_struct()
        return self.sequence_struct()


    def run(self):
        """
        main run loop
        """

        print "Digital Environment is running"
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                r.sleep()
            except KeyboardInterrupt:
                print "\n Environment module turned off"
                break


if __name__=="__main__":
    env = Environment()
    env.create_a_struct()
