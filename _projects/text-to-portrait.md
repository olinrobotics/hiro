---
date: 2019-03-19T04:00:00.000+00:00
layout: project
title: text-to-portrait
sub_heading: Allow an UR5 arm to draw any picture a user wants.
tags:
- Fall 2019
banner_image: "/hiro/uploads/preview.jpg"
description: Text to portrait is a script that allows users to input what image they
  want Castor or Pollux (UR5 arms) to draw. Using the Google custom search API, it
  downloads the first 10 images of a specific search query. It then uses OpenCV to
  extract the edges of the images and translates it to coordinates for the arm to
  move in. A GUI is also displayed to help users select which image to draw and to
  control certain parameters like thresholds and drawing speed.
slug: ''
image_slider: []
members:
- 'Richard Gao  '
- HK Rho

---
# text-to-portrait

Allow an UR5 arm to draw any picture that a user wants.

## Description

Text to portrait is a script that allows users to input what image they want Castor or Pollux (UR5 arms) to draw. Using the google custom search api, it downloads the first 10 images of a specific search query. It then uses opencv to extract the edges of the images and translates it to coordinates for the arm to move in. A gui is also displayed to help users select which image to draw and to control certain parameters like thresholds and drawing speed.

![](/hiro/uploads/pikachu_1.png)

![](/hiro/uploads/pikachu_2.png)

![](/hiro/uploads/katamari.png)

## Demo Video

<iframe width="100%" height="415" src="https://www.youtube.com/embed/Hc0N01buptI" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## Tools Used

* Ubuntu 16.04
* ROS kinetic
* Python
* OpenCV
* Matplotlib