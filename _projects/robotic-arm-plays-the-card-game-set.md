---
date: 2017-12-19T05:00:00.000+00:00
layout: project
title: Robotic Arm Plays the Card Game Set
sub_heading: In this project, we programmed a robotic arm to identify and pick up
  3 cards that make a set.
tags:
- Fall 2017
banner_image: "/hiro/uploads/post_set_2.jpg"
description: ''
slug: ''
image_slider: []
members:
- Cassandra Overney
- Enmo Ren
- " Khang Vu "
- Emma Pan

---
# Description

In this project, we programmed a robotic arm to identify and pick up 3 cards that make a set. Our work involved image processing, image analysis, and robotics programming in ROS.

# How it works

## Algorithm to find a set

The basic idea is to look through all possible combinations of 3 cards and check whether they make a set. The algorithm is encapsulated into a class, which can be found [here](https://github.com/olinrobotics/irl/blob/master/irl_archive/Fall_2017/set/scripts/Set.py).

## Robotic arm's actuation

In this project, the arm will first move to the center position above the Set board to take a good picture of the board. After processing the picture and finding 3 cards that make a set, the arm would then move to those cards and pick them up.

![Robotic arm ready to play the Set game](https://minhkhang1795.github.io/img/post_set_1.jpg "Robotic arm ready to play the Set game")

_Robotic arm ready to play the Set game_