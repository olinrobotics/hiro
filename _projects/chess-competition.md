---
date: 2018-12-18 05:00:00 +0000
layout: project
title: Chess Competition
sub_heading: Chess-Competition uses a simplified method to create a chess game experience
  between a UR5 robot arm and a human player.
tags:
- Fall 2018
banner_image: "/hiro/uploads/chess_competition.JPG"
description: ''
slug: ''
image_slider: []
members:
- Audrey Lee
- SeungU Lyu

---
# Description

Chess-Competition uses a simplified method to create a chess game experience between a UR5 robot arm and a human player. A perception process was created in order to mitigate the error during the piece recognition process by using OpenCV to recognize different colors of the chess pieces.

# Prerequisites

Read [Install ROS Wrapper 2.0 for Intel RealSense Devices](https://github.com/olinrobotics/hiro/wiki/Tutorial:-Install-ROS-Wrapper-2.0-for-Intel-RealSense-Devices) for the full instruction.

* librealsense: [https://github.com/IntelRealSense/librealsense]()
* realsense2_camera: [https://github.com/intel-ros/realsense]()
* rgbd_launch: [https://github.com/ros-drivers/rgbd_launch.git]()
* starfish UCI chess engine: [https://github.com/official-stockfish/Stockfish]()
* python-chess: [https://github.com/niklasf/python-chess]()
* robotiq 2F-140 Gripper: [https://github.com/ros-industrial/robotiq]()

# Run Program

Connect with the gripper and the UR5 arm

```bash
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=10.42.0.175
rosrun hiro ur5_arm_node.py _robot_ip:=10.42.0.175
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
(optional) rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py
```

Run Chess Competition python script

```bash
python movement.py
```

# Project Description

Chess Competition is a program that enables robotic arms to play chess against a human player. Using an RGB picture of the current board layout, it masks all colors except the specific blue and red of the chess pieces to locate the positions of those chess pieces. It then determines its moves based on the chess pieces' positions on the board. The program stops when either the player or the robot wins.

## Between Turns

![Between Turns](hiro_active/projects/chess/BetweenTurns.png)

In Chess-Competition, there are three different forms of information about the chessboard used. The first form is in the form of a virtual chessboard inside the program, written with FEN (Forsyth–Edwards Notation), so that the chess engine can recognize the position of each chess piece later on. The second form is the actual chessboard, in which the program stores information about the real game situation. The last form is a simplified chessboard which is stored as an 8x8 matrix. This form stores color information of each position.

## Human Turn

![Human Turn Step 1](hiro_active/projects/chess/HumanTurn.png)

When human player moves a piece, the information on the actual chessboard is converted into a new simplified chessboard by using OpenCV. OpenCV recognizes different colors on each position of the actual chessboard, and saves them into a single 8*8 matrix.

![Human Turn Step 2](hiro_active/projects/chess/HumanTurn2.png)

By comparing the new simplified chessboard with the one from the previous turn, the program figures out which movement is made by the human player. From the example above, the human player moves “f7f5”, meaning a piece from the cell f7 is moved to the cell f5.

![Human Turn Step 3](hiro_active/projects/chess/HumanTurn3.png) The movement is then saved to the virtual chessboard (written with FEN), by the program running the command “execute movement f7f5” to the virtual chessboard.

## Robot Turn

![Robot Turn Step 1](hiro_active/projects/chess/RobotTurn.png) The program uses an open-source chess engine called “Stockfish 10” to calculate the next best movement from the current chessboard information. By entering the virtual chessboard information retrieved at the end of the human turn to the engine, it calculates the best movement that can be made by the robot. From the above chessboard, Stockfish calculated that “g1f3” would be the best movement, and is then executed.

![Robot Turn Step 2](hiro_active/projects/chess/RobotTurn2.png) The UR5 robot arm then updates the actual chess board to match the virtual chessboard.

# Documentation

## `OpenCvRealSenseCameras.py`

This is the script for the chess pieces detection. This script will:

* Initialize the camera
* Get the RGB stream from the camera
* Take a picture of the current scene from the camera
* Isolate the red and blue colors of the chess pieces
* Finds the center coordinates of the chess pieces
* Checks if chess pieces are in spaces on the chessboard

### `color_range()`

This function calculates the acceptable range to generate a mask to only show the specified color.

### `color_detect()`

This function masks the image and reveals only the color needed. It then finds the contours of the shown shapes of color and finds the center points of those contours.

### `thresh_callback()`

Calls `color_range()` and `color_detect()`.

### `Grid()`

This function checks each square on the chessboard and determines if there is a red or blue object in the squares.

## `movement.py`

This is the master script that moves the arm and opens the gripper to pick up and move the chess pieces. This script will:

* Have own data chessboard used inside the program.
* Use OpenCV to recognize the physical position of chess pieces after a human move from `OpenCvRealSenseCameras.py` script.
* Compare the position of chess pieces on data chessboard and the physical chessboard, and recognize which piece moved to which position.
* Not require the need of recognizing the exact types and positions of each chess piece.
* Use stockfish chess engine 10 and python-chess script to calculate the next best move for the robot arm.
* Use the calculated next move and physically move the chess pieces by the robot arm.
* End the program when the chess game is over.

# Tools Used

* Ubuntu 16.04
* ROS Kinetic
* Python
* OpenCV
* Stockfish