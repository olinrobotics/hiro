#!/usr/bin/env python
"""
Plot helpers to visualize the structure
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import numpy as np


def plot_cube3d(coords):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(coords[:, 0], coords[:, 1], coords[:, 2], zdir='z', c='red')
    plt.axis('equal')
    plt.show()


def plot_cube2d(cubes):
    plt.plot(cubes[:, 0], cubes[:, 2], 'ro')
    plt.axis('equal')
    plt.show()


def reduce_coords(coords):
    c = []
    for i, coord in enumerate(coords):
        x, y, z = coord
        y = -y
        if not np.math.isnan(x) and z < 1:
            if i % 10 != 0:
                continue
            c.append(coord)

    return np.asarray(c)


def plot_structure(cubes, cube_size=0.037, color='c'):
    """Plots the obstacle course that is generated"""

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plt.rcParams["figure.figsize"] = (12, 12)

    for cube in cubes:
        x, z, y = cube
        ax.bar3d(x * 10, y * 10, (z - cube_size / 2) * 10, cube_size * 10, cube_size * 10, cube_size * 10, color,
                 alpha=0.8)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_zlim(0, 2)
    plt.xlim(-1, 1.5)
    plt.ylim(4.2, 7)
    plt.show()


if __name__ == '__main__':
    plot_structure([[0, 0, 0]])
    # coords = np.loadtxt('coords_1.txt', dtype=float)
    # coords = reduce_coords(coords)
    # plot_cube3d(coords)
