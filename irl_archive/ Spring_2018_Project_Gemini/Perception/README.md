# Perception
## Prerequisites
Read [Install ROS Wrapper 2.0 for Intel RealSense Devices](https://github.com/olinrobotics/irl/wiki/Install-ROS-Wrapper-2.0-for-Intel-RealSense-Devices)
for the full instruction.
* librealsense: https://github.com/IntelRealSense/librealsense
* realsense2_camera: https://github.com/intel-ros/realsense
* rgbd_launch: https://github.com/ros-drivers/rgbd_launch.git

To run `test()` function in `perception.py` and plot the results in 3D, install `matplotlib`: https://matplotlib.org

## Documentation
### ```perception.py```
This is the master script of cube detection and localization for a structure of cubes. The cube size is 3.7 cm.
This script will:
* Initialize the depth camera
* Get point clouds and the RGB stream from the camera
* Call `skin_detection.py` to check the presence of human hands
* If hands are not present, call `find_paper_coords()` function 
to get the coordinates of the table (white pixels in the RGB image)
* Using the table coordinates, call `find_height_angle()` to calculate 
the height and the angle of depression of the camera
* Having the height and the angle, perform transformation so that
the coordinate system of the camera aligns with that of the real world
* Using the transformed coordinates, call `localization.py` to localize the cubes
* Publish the result to the **perception** topic

#### ```find_paper_coords()```
This functions performs paper detection using color detection of white given an image from 
the intel realsense camera and output 3D point cloud of the paper (the table where we set up the cube structure)

#### ```find_height_angle()```
Given the point cloud of the paper (the flat surface on which we build the structure) and the
way the camera is positioned, we want to find the depression angle of the camera
and its height. First, we take 3 random linearly independent points of the paper
to form a normal vector of the flat surface. Using the normal vector, we then compute the angle 
of depression of the camera. In order to find the height, we just need to find the distance between
the origin (aka the camera) to the surface defined by the 3 random points above.  

### ```skin_detection.py```
Given an array of RGB pixels, the presence of a hand is determined using a range of HSV pixel intensities that could be
considered as skin. A `skinMask` is then created to isolate the relevant pixels and `has_hand()` returns true if the count
reaches a certain threshold.

Credits to the `pyimagesearch` tutorial on skin detection

### ```localization.py```
This script performs localization of each cube, given a point cloud of the entire structure. The xyz coordinate system of the point cloud has been transformed to match the xyz coordinate system of the world to offset the effect of camera tilting at some nonzero angle. After the transformation, the y axis represents the up and down direction, which is the height of the structure, the z axis represents forward and backward direction, which is the depth of the structure with respect to the camera, and the x asis represents the left and right direction, which is width of the structure. Given the point cloud of the structure, we then execute the following steps to find the xyz coordinate of each cube with respect to the world:


##### Step 1: ```reduced_coords(coords, cube_size)```
* Since the camera is mounted at some height, we limit the maximum height (y) of the point cloud and ignore any points whose y coordinates is greater than 5 times the height of each cube.
* To improve accuracy, we set a maximum z value (depth) and only focus on point clouds with z coordinate smaller than the threshold. Points with depth greater than this threshold are most likely to be noise.

##### Step 2:
We then check the cube by height. We define the height of the top surface of the cubes at each floor level with some tolerance of error as the level. For example, the height (y) of the first level would equal to the height of the cube. The result is a slice of point clouds at each level where the thickness of the slice is the tolerance of error.

##### Step 3: ```find_cubes_at_height(coords, height_level, cube_size)```
* For each slice, we compute the minimum x value, maximum x value, minimum z value and maximum z value of the structure and define a bounding box. The coordinates of the four corners of the bounding box are (min_x, max_z), (min_x, min_z), (max_x, max_z), (max_x, min_z).  
* We then sort the point cloud at each slice by depth and divide the slice into strips where the cubes in each strip have the same depth (z) with respect to the camera.
* Lastly, we scan each strip starting from the points with minimum x coordinates and progress to the right by increments of the cube size. The strip of point clouds is divided into multiple regions of square shape to allow for checking the presence of a cube.

##### Step 4: ```check_cubes(coords, height_level, cube_size)```
* Give a confined region of square shape as defined in **Step 3**, we first scanned the number of points in the point cloud. If the number of points in the region is less than threshold, we categorized that region as no cubes present. The threshold is currently defined to be
* If the region have number of points greater than the threshold, we then check the area formed by the point cloud is above certain threshold.
* Lastly, we check the presence of a hole.

## Demo
Youtube link: https://www.youtube.com/watch?v=fltjSKhnXMo

## Authors
* [**Cassandra Overney**](https://github.com/coverney)
* [**Enmo Ren**](https://github.com/Enmoren)
* [**Khang Vu**](https://github.com/minhkhang1795)
* [**Sherrie Shen**](https://github.com/xieruishen)