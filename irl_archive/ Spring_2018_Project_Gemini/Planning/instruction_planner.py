#!/usr/bin/env python

"""
Project Gemini Instruction Planner

by Kevin Zhang

The Instruction Planner that holds the RL and systematic sequencer which can
take a group of cubes and "sort" them into a comprehensive set of instructions
for the controller to use

You just rosrun this thing:

rosrun irl instruction_planner

How it goes:
1. Waits for data from perception team
2. takes the data and transforms it into GridWorld format (0-4)
3. finds any differences (in case of consecutive readings)
4. sorts the Grid Cubes using RL and systematic
5. converts the sorted list to real world coordinates (deprecated)
6. publishes the message to Controller for physical execution
"""

import rospy
import rospkg
import numpy as np
import pandas as pd
import random
import time
import itertools
from irl.msg import Real_Cube, Grid_Cube, Real_Structure, Grid_Structure, Cube_Structures
from std_msgs.msg import String, Int16, Bool
from Assembler.cube import Digital_Cube

from Assembler.assembly_instructor import Assembler
from Transformation.coordFrames import CoordFrames

class Planner(object):
    """
    the planner class, which holds all of the components that comprise of the "brain" functionality in
    figuring out what to do with the data that perception provides, and then sends its instructions
    to the controller for physical execution
    """

    def __init__(self):
        self.asm = Assembler()

        self.coord_trans = CoordFrames()
        # self.change_origin(-.065, .600, .0185)

        self.cube_list = Real_Structure() # the premlinary cube list input used to model the env
        self.cubes = Grid_Structure() # the cube list output used to sort the cubes
        self.two_structs = Cube_Structures()
        self.env_size = 5 # dimension of env
        self.env_diffs = np.empty((self.env_size,self.env_size,self.env_size), dtype=object) # the digital environment
        self.current_env = np.empty((self.env_size,self.env_size,self.env_size), dtype=object) # the previous digital environment

        # sorted cubes in both grid format and real coordinate format, the real one is deprecated through
        self.sorted_grid_cubes = None
        self.sorted_real_cubes = None

        # ROS node stuff
        rospy.init_node("instruction_planner")
        # getting data from perception
        rospy.Subscriber("/perception", Real_Structure, self.plan)
        # sending instructions to the controller
        self.instructions_pub = rospy.Publisher("/build_cmd_castor", Cube_Structures, queue_size=10)
        self.status_pub = rospy.Publisher("/controller_status", Bool, queue_size=10)


    def change_origin(self, px, py, pz):
        """
        small updater to change the calibration of the minimap cubes, used by the
        coordinate transformer
        """

        origin_cube = Real_Cube()
        origin_cube.x = px
        origin_cube.y = py
        origin_cube.z = pz

        self.coord_trans.updateOrigin(origin_cube)


    def plan(self, cube_list):
        """
        the main planning method

        takes the cubes from perception and sends it to coordinate transforms to turn it into
        Grid World.
        finds any differences between this reading and the previous reading
        then sort the differences with RL and systematic
        format for physical world implementation,
        and publish to Controller
        """

        # get the cubes
        self.cube_list.building = cube_list.building

        # format to Grid World
        env = self.coord_trans.convertBoard(self.cube_list)

        # find any differences between this read environment and the last one, and
        # determine if there needs to be anything built
        need_build = self.find_diffs(env)

        if need_build:

            # add descriptors for sorting
            self.add_descriptors()

            # sort into instructions
            self.sorted_grid_cubes = self.sequence()

            # convert into real cubes (deprecated)
            self.sorted_real_cubes = self.coord_trans.convertReal(self.sorted_grid_cubes)

            # builds message
            self.two_structs.real_building = self.sorted_real_cubes
            self.two_structs.grid_building = self.sorted_grid_cubes


            # publish to Controller
            self.instructions_pub.publish(self.two_structs)
            for x, y, z in itertools.product(*map(xrange,(self.env_size, self.env_size, self.env_size))):
                if self.env_diffs[x][y][z]:
                    self.current_env[x][y][z] = self.make_grid_cube(self.env_diffs[x][y][z])
        else:
            # else just do nothing and let the perception read again
            print "NOTHING NEW, NO NEED TO BUILD THIS READING"
            self.status_pub.publish(False)


    def find_diffs(self, environment):
        """
        find the differences between the current and previous readings, and output
        whether something needs to built
        """

        self.env_diffs = np.empty((self.env_size,self.env_size,self.env_size), dtype=object)

        # goes through the newly read environment and sees if there are any differences
        # between it and the known environment
        for x, y, z in itertools.product(*map(xrange,(self.env_size, self.env_size, self.env_size))):
            if environment[x][y][z] != self.current_env[x][y][z]:
                self.env_diffs[x][y][z] = self.make_grid_cube(environment[x][y][z])

        # checks to see if anything needs to be built
        # note that this will also say nothing needs to be built for removed cubes,
        # as they show up as diffs but self.env_diffs is set to None there, since now there's
        # nothing there. This is fine for the current idea of the presentation, as
        # we're not planning on allowing deletions, only additions
        if all(self.env_diffs[x][y][z] == None for \
            x, y, z in itertools.product(*map(xrange,(self.env_size, self.env_size, self.env_size)))):
            return False
        else:
            return True


    def add_descriptors(self):
        """
        adds descriptors for center and ring, as described in assembly_instructor
        used in sorting the cubes into an instruction set
        """

        # the cubes to be sorted
        self.cubes = Grid_Structure()

        # initializes the center and ring environments
        current_env_center = np.empty((self.env_size,self.env_size,self.env_size), dtype=object)
        current_env_ring = np.empty((self.env_size,self.env_size,self.env_size), dtype=object)

        # populates them from the current_env
        for x, y, z in itertools.product(*map(xrange,(self.env_size, self.env_size, self.env_size))):
            if x < 4 and x > 0 and y < 4 and y > 0:
                if self.env_diffs[x][y][z]:
                    current_env_center[x][y][z] = self.make_grid_cube(self.env_diffs[x][y][z])
            else:
                if self.env_diffs[x][y][z]:
                    current_env_ring[x][y][z] = self.make_grid_cube(self.env_diffs[x][y][z])

        # make actual usable cubes from the environment and filling out all the information
        # this is describes the center and the ring in a vacuum, so they don't interfere with each other
        for x, y, z in itertools.product(*map(xrange,(self.env_size, self.env_size, self.env_size))):
            if current_env_center[x][y][z]:
                connections = 0
                for c in [[x+1, y],[x-1, y], [x,y+1], [x,y-1]]:
                    if all(n >= 0 and n < self.env_size for n in c) and current_env_center[c[0]][c[1]][z]:
                        connections += 1
                current_env_center[x][y][z].connections = connections
                current_env_center[x][y][z].height = current_env_center[x][y][z].z + 1

            elif current_env_ring[x][y][z]:
                connections = 0
                for c in [[x+1, y],[x-1, y], [x,y+1], [x,y-1]]:
                    if all(n >= 0 and n < self.env_size for n in c) and current_env_ring[c[0]][c[1]][z]:
                        connections += 1
                current_env_ring[x][y][z].connections = connections
                current_env_ring[x][y][z].height = current_env_ring[x][y][z].z + 1

        # recombine the two halves into one environment
        for x, y, z in itertools.product(*map(xrange,(self.env_size, self.env_size, self.env_size))):
            if x < 4 and x > 0 and y < 4 and y > 0 and current_env_center[x][y][z]:
                self.cubes.building.append(current_env_center[x][y][z])
            elif current_env_ring[x][y][z]:
                self.cubes.building.append(current_env_ring[x][y][z])


    def make_grid_cube(self, cube):
        """
        converting between data types, this one converts from python cube to ros cube
        """
        if not cube:
            return None
        grid_cube = Grid_Cube()
        grid_cube.x = cube.x
        grid_cube.y = cube.y
        grid_cube.z = cube.z
        return grid_cube


    def sequence(self):
        """
        sequence wrapper for RL and systematic sequencer
        """

        self.asm.set_cube_list(self.cubes)
        return self.asm.sequence()


    def run(self):
        """
        main run loop
        """

        print "Planner is running"
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                r.sleep()
            except KeyboardInterrupt:
                print "\n Planner module turned off\n"
                break


if __name__=="__main__":
    p = Planner()
    p.run()
