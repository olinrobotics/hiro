#!/usr/bin/env python

"""
Benjmain Ziemann
benjamin.ziemann@students.olin.edu

Recieves cube struct from Kevin containing

"""

from __future__ import absolute_import
import numpy
import math
from irl.msg import Grid_Cube, Real_Cube, Real_Structure, Grid_Structure, Cube_Structures
import rospy
import rospkg

class CoordFrames(object):

    def __init__(self):
        """"
        Setup and variables associated with coordinate frames
        """

        rospack = rospkg.RosPack()
        self.PACKAGE_PATH = rospack.get_path("irl")
        #Cube for calibration  due to camera shifting
        f = open(self.PACKAGE_PATH+"/projects/Planning/Transformation/previousOrigin.txt", "r")
        coords = f.readlines()
        f.close()
        self.origin = Real_Cube()
        self.origin.x = float(coords[0].strip())
        self.origin.y = float(coords[1].strip())
        self.origin.z = float(coords[2].strip())

        #Physical offsets of global origin
        self.armOffSetY = -.550
        self.polluxOffSetX = -.005
        self.polluxOffSetY = -.015
        self.castorOffSetY = -.545
        self.castorOffSetX = -.01
        self.cubeSize = .0897

        # the currently accepted calibration, gonna keep these around for a while
#         self.pixelX = [-.065, -.025, .015, .055, .095]
# -       self.pixelY = [.600, .560, .520, .480, .440]
# -       self.pixelZ = [.0185, .0585, .0985, .139, .180]

        #Camera values
        self.pixelX = []
        self.pixelY = []
        self.pixelZ = []

        for i in range(5):
            self.pixelX.append(float('%.3f'%(self.origin.x+(i*0.04))))
        for i in range(5):
            self.pixelY.append(float('%.3f'%(self.origin.y-(i*0.04))))
        for i in range(5):
            self.pixelZ.append(float('%.3f'%(self.origin.z+(i*0.04))))

        #Real world values - Note arm coords are orthogonal to board coords.
        #Castor set
        # self.realYC = [2.0*self.cubeSize+self.castorOffSetY, self.cubeSize+self.castorOffSetY, self.castorOffSetY, 0.0, 0.0]
        # self.realXC = [2.0*self.cubeSize+self.castorOffSetX, self.cubeSize+self.castorOffSetX, self.castorOffSetX, -self.cubeSize+self.castorOffSetX, -2.0*self.cubeSize+self.castorOffSetX]

        # Castor new set
        self.realYC = [-0.3400, -0.4380, -0.5300, 0.0, 0.0]
        self.realXC = [0.1720, 0.0811, -0.0113, -0.1047, -0.1943]

        #Pollux Set
        # self.realYP = [0.0, 0.0, 0.0, self.armOffSetY+self.cubeSize+self.polluxOffSetY, self.armOffSetY+2.0*self.cubeSize+self.polluxOffSetY]
        # self.realXP = [-2.0*self.cubeSize+self.polluxOffSetX, -self.cubeSize+self.polluxOffSetX, self.polluxOffSetX, self.cubeSize+self.polluxOffSetX, 2.0*self.cubeSize+self.polluxOffSetX]

        # Pollux new Set
        self.realYP = [0.0, 0.0, 0.0, -0.4579, -0.3635]
        self.realXP = [-0.1871 ,-0.0938 , -0.0011, 0.0907, 0.1808]

        #Shared
        self.realZP = [.197+.07, .291+.07, .387+.07, .479+.07, .573+.07]
        self.realZC = [0.242, 0.336, 0.43, 0.524, 0.618]



    def updateOrigin(self, originCube):
        """
        Updates the origin cube that is used to make the board

        Use for calibration purposes.
        originCube is a Grid_Cube()
        """
        self.origin = originCube

        self.pixelX = []
        self.pixelY = []
        self.pixelZ = []

        for i in range(5):
            self.pixelX.append(float('%.3f'%(self.origin.x+(i*0.04))))
        for i in range(5):
            self.pixelY.append(float('%.3f'%(self.origin.y-(i*0.04))))
        for i in range(5):
            self.pixelZ.append(float('%.3f'%(self.origin.z+(i*0.04))))

        f = open("previousOrigin.txt", "w")
        f.truncate()
        f.write(str(self.origin.x)+"\n")
        f.write(str(self.origin.y)+"\n")
        f.write(str(self.origin.z)+"\n")
        f.close()


    def closest(self, values, val):
        """
        Returns the element from a list's index that is
        closest to the given value
        """
        mini = 999999
        min_index = None
        for i in range(5):
            if math.fabs(val-values[i]) < mini:
                mini = math.fabs(val-values[i])
                min_index = i
        return min_index


    def convertBoard(self, cubes):
        """
        Takes in a list of cubes structs and converts it to board model

        Called by Kevin's script for building planning
        """
        #Set Up board
        board = []

        #Add a new z layers
        for i in xrange(5):
            board.append([[None, None, None, None, None],
                            [None, None, None, None, None],
                            [None, None, None, None, None],
                            [None, None, None, None, None],
                            [None, None, None, None, None]])

        #Convert cube data to board dimensions
        #Fill out board according to cube data
        for cube in cubes.building:
            gcube = Grid_Cube()
            gcube.x = self.closest(self.pixelX, cube.x)
            gcube.y = self.closest(self.pixelY, cube.y)
            gcube.z = self.closest(self.pixelZ, cube.z)
            board[gcube.x][gcube.y][gcube.z] = gcube
        return board #3d array with x, y, and z of blocks


    def convertReal(self, cubes):
        """
        Takes in a list of cubes structs and converts it to a real board

        Called by Kevin's script for building planning
        """

        #Fill out board according to cube data
        real_cubes = Real_Structure()
        for cube in cubes.building:
            real_cube = Real_Cube()
            if(cube.y < 3):
                real_cube.x = self.realXC[cube.x]
                real_cube.y = self.realYC[cube.y]
                real_cube.z = self.realZC[cube.z]
            else:
                real_cube.x = self.realXP[cube.x]
                real_cube.y = self.realYP[cube.y]
                real_cube.z = self.realZP[cube.z]

            real_cubes.building.append(real_cube)
        return real_cubes

if __name__ == "__main__":
    cf = CoordFrames()
