#!/usr/bin/env python

import numpy as np
import time
import sys
import rospy
import Tkinter as tk
import random
from irl.msg import minimap, blocks
from std_msgs.msg import String

"""

The UI made for the mvp version of Project Gemini
by Kevin Zhang

It opens a graphical interface that allows users to input a specific structure, then
upon clicking a button will send the information to the brain/planner, which will go
from there
"""

UNIT = 100   # pixels
MAZE_H = 5  # grid height
MAZE_W = 5  # grid width


class Cell(object):
    """
    Cell class, which holds one cell in the grid. Can be switched on or off
    depending on whether the user wants to put a "block" there or not
    """

    def __init__(self, canvas, x, y, size):
        self.canvas = canvas
        self.abs = x
        self.ord = y
        self.size= size
        self.fill= False
        self.rect = None

        self.xmin = self.abs * self.size
        self.xmax = self.xmin + self.size
        self.ymin = self.ord * self.size
        self.ymax = self.ymin + self.size


    def switch(self):
        """
        Switch if the cell is filled or not.
        """
        self.fill= not self.fill


    def draw(self):
        """
        order to the cell to draw its representation on the canvas
        """

        if self.canvas != None :
            fill = "#66d9ef" if self.fill else "#104494"

            self.rect = self.canvas.create_rectangle(self.xmin, self.ymin, self.xmax, self.ymax, fill = fill, activefill="#a1e9f7")
            self.canvas.tag_bind(self.rect, "<Button-1>", self.handleMouseClick)


    def handleMouseClick(self, event):
        self.switch()
        self.canvas.delete(self.rect)
        self.draw()


class UI(tk.Tk, object):
    """
    The UI class, which builds up a UI with cells and then waits for a structure to send off
    """

    def __init__(self):
        super(UI, self).__init__()
        self.title('Minecraft MVP')
        rospy.init_node("mvp_ui")
        self.pub = rospy.Publisher("minimap", minimap, queue_size=10)
        self.build_env()


    def build_env(self):
        """
        makes a UI
        """

        # base
        self.canvas = tk.Canvas(self, bg='white',
                           height=MAZE_H * UNIT,
                           width=MAZE_W * UNIT)

        # cells
        self.grid = []
        for row in range(5):
            line = []
            for column in range(5):
                line.append(Cell(self.canvas, column, row, UNIT))
            self.grid.append(line)

        # labels
        self.label = tk.Label(self, text="Use the grid to plan your structure, then click BUILD when done!", width="50", height="5", bg='#1a0f35', fg="white")
        self.label.pack(side=tk.TOP, padx=0, pady=0, fill= tk.X)
        self.draw()

        # buttons
        self.b = tk.Button(self, text="Build", width="50", height="5", bg="#0483e9", activebackground="#a1e9f7")
        self.b.bind("<Button-1>", self.callback)
        self.b.pack(side=tk.BOTTOM, padx=0, pady=0, fill=tk.X)

        # pack all
        self.canvas.pack()


    def callback(self, event):
        """
        callback on button press, sends out the information
        """

        print "Begin Processing"
        cubes = []
        name = 0
        for row in range(5):
            for column in range(5):
                if self.grid[row][column].fill:
                    block = blocks()
                    block.x, block.y, block.name = self.grid[row][column].xmin, self.grid[row][column].ymin, str(name)
                    cubes.append(block)
                    name += 1
        random.shuffle(cubes)

        msg = minimap()
        msg.structure = cubes
        self.pub.publish(msg)


    def draw(self):
        """
        rendering the UI
        """

        for row in self.grid:
            for cell in row:
                cell.draw()


if __name__ == '__main__':
    env = UI()
    env.mainloop()
