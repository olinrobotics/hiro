---
date: 2017-12-19T05:00:00.000+00:00
layout: project
title: Robotic Arm Solves Sudoku
description: We programmed a robotic arm to solve a 4x4 Sudoku puzzle by itself (ROS
  in Python).
tags:
- Fall 2017
banner_image: "/hiro/uploads/post_sudoku_1.jpg"
sub_heading: ''
slug: ''
image_slider: []
members:
- Khang Vu
- Nick Steelman
- Sherrie Shen

---
# Description

We programmed a robotic arm to solve a 4x4 Sudoku puzzle by itself (ROS in Python). The robot can be stopped anytime by the user using a built-in command interface.

<iframe width="100%" height="415" src="https://www.youtube.com/embed/idnPBx4LdmI" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

<iframe width="100%" height="415" src="https://www.youtube.com/embed/8wan8w3xcxE" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

# How it works

## Algorithm to solve a Sudoku puzzle

An algorithm to solve a sudoku puzzle was inspired by the paper: [A Pencil-and-Paper Algorithm for Solving Sudoku Puzzles](https://www.ams.org/notices/200904/tx090400460p.pdf).

You can find the algorithm written in Python here: [Sudoku algorithm](https://github.com/olinrobotics/irl/tree/master/irl_archive/Fall_2017/sudoku_solver/scripts/sudoku_algorithm).

![Printing sudoku board](https://minhkhang1795.github.io/img/post_sudoku_6.jpg "Printing sudoku board")

_A 4x4 sudoku board used in this project (24" x 24")_

## Capturing the board and processing the digits

The picture taken by the robotic arm would be processed using [adaptive thresholding techniques](https://docs.opencv.org/3.3.1/d7/d4d/tutorial_py_thresholding.html) in OpenCV.

![Sudoku board taken from the robotic arm](https://minhkhang1795.github.io/img/post_sudoku_7.jpg "Sudoku board taken from the robotic arm")

_Sudoku board taken from the robotic arm_

We then find the contour of the Sudoku board from the processed image and apply [perspective transformation](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_geometric_transformations/py_geometric_transformations.html#perspective-transformation) on the contour to get a perfect-square image of the Sudoku grid.

![Processed image after adaptive thresholding and perspective transformation](https://minhkhang1795.github.io/img/post_sudoku_9.jpg "Processed image after adaptive thresholding and perspective transformation")

_Processed image after adaptive thresholding and perspective transformation_

Finally, we crop out each digit and apply [k-NN algorithm in OpenCV](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_ml/py_knn/py_knn_understanding/py_knn_understanding.html#knn-in-opencv) to predict the digits.

![Final image processing result](https://minhkhang1795.github.io/img/post_sudoku_8.jpg "Final image processing result")

_Final image processing result_

Using the Sudoku algorithm above, we'll find a solution for this puzzle.

![A solved Sudoku puzzle](https://minhkhang1795.github.io/img/post_sudoku_10.jpg "A solved Sudoku puzzle")

_A solved Sudoku puzzle_

## Actuation of the Robotic Arm

In this project, the arm will move to the center position above the Sudoku board to take a good picture of the board. After processing the picture and solving the Sudoku puzzle, the arm would then move to each empty cell and write the solution.

![The arm is writing digits on the board](https://minhkhang1795.github.io/img/post_sudoku_5.jpg "The arm is writing digits on the board")

_The arm is writing digits on the board_

![The arm has solved the puzzle](https://minhkhang1795.github.io/img/post_sudoku_11.jpg "The arm has solved the puzzle")

_The arm has solved the puzzle_

Check out the [main function](https://github.com/olinrobotics/irl/blob/master/irl_archive/Fall_2017/sudoku_solver/scripts/sudoku_main.py#L358) to see how the arm moves.