---
date: 2017-12-19 05:00:00 +0000
layout: project
title: Robotic Arm Solves Sudoku
sub_heading: We programmed a robotic arm to solve a 4x4 Sudoku puzzle by itself (ROS
  in Python).
tags:
- Fall 2017
banner_image: "/uploads/post_sudoku_1.jpg"
description: ''
slug: ''
image_slider: []
members:
- Khang Vu
- Nick Steelman
- Sherrie Shen
published: false

---
# Description

We programmed a robotic arm to solve a 4x4 Sudoku puzzle by itself (ROS in Python). The robot can be stopped anytime by the user using a built-in command interface.

<iframe width="100%" height="415" src="https://www.youtube.com/embed/idnPBx4LdmI" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

# How it works

## Algorithm to solve a Sudoku puzzle

An algorithm to solve a sudoku puzzle was inspired by the paper: [A Pencil-and-Paper Algorithm for Solving Sudoku Puzzles](https://www.ams.org/notices/200904/tx090400460p.pdf).

You can find the algorithm written in Python here: [Sudoku algorithm](https://github.com/olinrobotics/irl/tree/master/irl_archive/Fall_2017/sudoku_solver/scripts/sudoku_algorithm).

![Printing sudoku board](https://minhkhang1795.github.io/img/post_sudoku_6.jpg "Printing sudoku board")

## Capturing the board and processing the digits

The picture taken by the robotic arm would be processed using [adaptive thresholding techniques](https://docs.opencv.org/3.3.1/d7/d4d/tutorial_py_thresholding.html) in OpenCV.

![Sudoku board taken from the robotic arm](https://minhkhang1795.github.io/img/post_sudoku_7.jpg "Sudoku board taken from the robotic arm")

We then find the contour of the Sudoku board from the processed image and apply [perspective transformation](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_geometric_transformations/py_geometric_transformations.html#perspective-transformation) on the contour to get a perfect-square image of the Sudoku grid.

![Processed image after adaptive thresholding and perspective transformation](https://minhkhang1795.github.io/img/post_sudoku_9.jpg "Processed image after adaptive thresholding and perspective transformation")

Finally, we crop out each digit and apply [k-NN algorithm in OpenCV](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_ml/py_knn/py_knn_understanding/py_knn_understanding.html#knn-in-opencv) to predict the digits.

![Final image processing result](https://minhkhang1795.github.io/img/post_sudoku_8.jpg "Final image processing result")

Using the Sudoku algorithm above, we'll find a solution for this puzzle.

![A solved Sudoku puzzle](https://minhkhang1795.github.io/img/post_sudoku_10.jpg "A solved Sudoku puzzle")

## Actuation of the Robotic Arm

In this project, the arm will move to the center position above the Sudoku board to take a good picture of the board. After processing the picture and solving the Sudoku puzzle, the arm would then move to each empty cell and write the solution.

![The arm is writing digits on the board](https://minhkhang1795.github.io/img/post_sudoku_5.jpg "The arm is writing digits on the board")

The arm is writing digits on the board

![The arm has solved the puzzle](https://minhkhang1795.github.io/img/post_sudoku_11.jpg "The arm has solved the puzzle")

Check out the [main function](https://github.com/olinrobotics/irl/blob/master/irl_archive/Fall_2017/sudoku_solver/scripts/sudoku_main.py#L358) to see how the arm moves.