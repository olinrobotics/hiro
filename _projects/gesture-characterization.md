---
date: 2017-01-12 05:00:00 +0000
layout: project
title: Gesture Characterization
sub_heading: Enable Edwin to interact with a user by identifying the user's movement
  and classifying it.
tags: []
banner_image: "/hiro/uploads/Follow Me Banner.png"
description: ''
slug: ''
image_slider: []
members:
- Sophia Nielsen
- Kevin Zhang

---
# Description 

Gesture characterization is a script that enables Edwin to interact with a user by identifying the user's movement and classifying it. The characterization is determined using the k-nearest neighbours machine learning algorithm, which can classify motions into gesture groups. In order to implement this, we create bag files of certain gestures, which became the training data for the k-NN machine learning algorithm. 

<iframe width="100%" height="415" src="https://www.youtube.com/embed/xjIaE0qrZJk" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

One of the applications of this gesture characterization is for a game of Simon Says where Edwin is Simon. Using this script, Edwin can determine whether the user is correctly executing the command of Simon when they have a command with "Simon says". 

<iframe width="100%" height="415" src="https://www.youtube.com/embed/QB8nBHxtrKg" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## Tool Used:

 * Ubuntu 14.04

 * ROS-indigo

 * Python

# Technical Discussion: 

We used K-nearest neighbour as our machine learning algorithm. Before we start, every skeleton is moved so that its neck is on the origin. By doing that, we eliminate the spatial differences that can cause inaccuracy. We then take the x,y,z coordinates of each skeleton points as our features. After training the model with gesture samples, Edwin will be able to predict any future gesture inputs.

In order for Edwin to detect a continuous gesture, we ask the Edwin to detect a few key frames the gesture. If all the frames are present over time, Edwin knows that the player is doing the continuous gesture.