#!/usr/bin/env python
"""
By Khang Vu, Cassandra & Sherrie, 2018
Last Modified April 18, 2018

Given transformed coordinates and the cube's size,
this scripts estimates the locations of the cubes
"""
import numpy as np

CUBE_SIZE_SMALL = 0.037  # in meter
CUBE_SIZE_LARGE = 0.086  # in meter
GRID_HEIGHT = 0.00635  # in meter


def cube_localization(coords, cube_size=CUBE_SIZE_SMALL):
    """
    Main function: given point cloud coordinates and the cube's size
    Return the COMs of the cubes
    :param coords: 3D point cloud coordinates
    :param cube_size: size of the cube
    :return: numpy array of COM (x, y, z) of the cubes
    """
    cube_coords = []

    # Reduce coordinates by only focusing on cubes
    coords = reduced_coords(coords, cube_size)

    # Assume that the structure is not taller than 6 cubes
    height_level = 1
    while height_level <= 6:
        cubes, coords = find_cubes_at_height(coords, height_level, cube_size)
        cube_coords.extend(cubes)
        height_level += 1
    return np.asarray(cube_coords)


def reduced_coords(coords, cube_size):
    """
    Eliminate coordinates that are unlikely to form the cubes
    For now, limit the height of coordinates from 0.75 x cube_size to 5.25 x cube_size
    :param coords: full coordinates
    :param cube_size: size of the cube
    :return: reduced coordinates
    """
    r_coords = []
    for coord in coords:
        # y coordinates flips for the librealsense camera
        coord[1] = -coord[1]
        x, y, z = coord
        if 0.75 * cube_size + GRID_HEIGHT <= y <= 6.25 * cube_size + GRID_HEIGHT and z < 0.7 and -0.1 < x < 0.15:
            r_coords.append(coord)
    return np.asarray(r_coords)


def find_cubes_at_height(coords, height_level, cube_size):
    cubes = []

    # Find all coords at y level, and remove those coords from the reduced_coords
    coords_at_y = []
    i_remove_list = []
    for i, coord in enumerate(coords):
        if abs(coord[1] - GRID_HEIGHT - height_level * cube_size) <= cube_size / 3:
            coords_at_y.append(coord)
            i_remove_list.append(i)
    coords_at_y = np.asarray(coords_at_y)
    coords = np.delete(coords, i_remove_list, axis=0)

    # Sort the new coords in-place by z
    coords_at_y.view('float64,float64,float64').sort(order=['f2'], axis=0)
    while coords_at_y.size != 0:
        # Find all coords from z_min to z_min + cube_size, and remove those coords from coords_at_y
        coords_at_yz = []
        z_min = coords_at_y[0][2]
        i_remove_list = []
        for i, coord in enumerate(coords_at_y):
            x, y, z = coord
            if z - z_min <= cube_size:
                coords_at_yz.append(coord)
                i_remove_list.append(i)
            else:
                break
        coords_at_yz = np.asarray(coords_at_yz)
        coords_at_y = np.delete(coords_at_y, i_remove_list, axis=0)

        # Sort the new coords in-place by x
        coords_at_yz.view('float64,float64,float64').sort(order=['f0'], axis=0)
        while coords_at_yz.size != 0:
            # Find coords from x_min to x_min + cube_size, and remove those coords from coords_at_yz
            coords_at_yzx = []
            x_min = coords_at_yz[0][0]
            i_remove_list = []
            for i, coord in enumerate(coords_at_yz):
                x, y, z = coord
                if x - x_min <= cube_size:
                    coords_at_yzx.append(coord)
                    i_remove_list.append(i)
                else:
                    break
            coords_at_yz = np.delete(coords_at_yz, i_remove_list, axis=0)

            # After limit all the point cloud to coords_at_yzx,
            # check if those coords form the top surface of a pile of cubes
            new_cubes = check_cubes(coords_at_yzx, height_level, cube_size)
            cubes.extend(new_cubes)

    return cubes, coords


def check_cubes(coords, height_level, cube_size):
    """
    Check if the given coords can form the top surface of a cube.
    If so, return a list of cubes from the top level to the bottom
    :param coords: coords sorted by x that might make a cube
    :param height_level: height level
    :param cube_size: size of the cube
    :return: list of COMs of the cubes; empty list if there is no cube
    """
    cubes = []

    # 1st check: eliminate the case when the number of point cloud < 400
    if len(coords) < 400:
        return cubes
    min_z = max_z_left = max_z_right = None
    min_x, max_x = coords[0][0], coords[-1][0]

    # Find min_z, max_z_left, max_z_right
    for coord in coords:
        x, y, z = coord
        if min_z > z or min_z is None:
            min_z = z
        if abs(min_x - x) <= cube_size / 2 and (max_z_left <= z or max_z_left is None):
            max_z_left = z
        if abs(max_x - x) <= cube_size / 2 and (max_z_right <= z or max_z_right is None):
            max_z_right = z

    # 2nd check: max_z_left and max_z_right should not be much different.
    # If they are different, then the top surface area will be small, hence not a cube
    max_z = (max_z_left + max_z_right) / 2
    coord_area = abs(max_x - min_x) * abs(max_z - min_z)
    expected_area = cube_size * cube_size
    if coord_area >= 0.65 * expected_area:
        # 3rd check: eliminate the case when the number of center coords < 500
        cube_x = min_x + cube_size / 2
        cube_z = min_z + cube_size / 2
        min_x = cube_x - cube_size * 0.8
        max_x = cube_x + cube_size * 0.8
        min_z = cube_z - cube_size * 0.8
        max_z = cube_z + cube_size * 0.8
        count = 0
        for coord in coords:
            x, y, z = coord
            if min_x <= x <= max_x and min_z <= z <= max_z:
                count += 1
        print("count", count)

        # Now it's officially a valid top surface
        if count > len(coords) * 75 / 100:
            while height_level >= 1:
                cube_y = (height_level - 1) * cube_size + cube_size / 2
                cube = np.asarray([cube_x, cube_y, cube_z])
                cubes.append(cube)
                height_level -= 1
    return cubes


if __name__ == '__main__':
    coords = np.loadtxt('coords_7.txt', dtype=float)
    cubes = cube_localization(coords)
    print cubes, len(cubes), "cubes"
    import plot

    plot.plot_cube2d(cubes)
