---
date: 2017-01-29 05:00:00 +0000
layout: project
title: Stereoscopic Vision
sub_heading: Teach Edwin to recognize and remember objects
tags: []
banner_image: ''
description: ''
slug: ''
image_slider: []
members:
- Lydia Zuehsow
- Yoonyoung Cho (Jamie)

---
# Overview

The goal of this research was to intuitively teach Edwin to recognize and remember objects, thus creating a robust tool for dataset generation. This decreased the amount of time we would need to spend generating datasets of individual objects for any future machine learning.

In order to identify objects we were interested in "teaching" to Edwin, we developed a stereoscopic camera system that could locate an objects distance from Edwin. We then tagged any objects closer than a certain distance as "significant."

<iframe width="100%" height="100%" src="https://www.youtube.com/embed/wFORJR2kNos" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

# The Hardware

Starting this project, we weren't sure what features our prospective cameras would need.

We ultimately chose the Hardkernal oCam both because it was a relatively high quality camera (which could be focused to different distances) and because we had a pair of oCams from a previous project (allowing us to start experimenting  with vision right away.)

## Our Cameras

![Hardkernel Camera](/hiro/uploads/ocam.jpg "Hardkernel Camera")

* Hardkernel oCam
* Standard M12 lens
* 3.6mm focal length
* 65 deg. field of view (FOV)
* 120 frames per second (fps) at a resolution of 640x480 pixels
* 5 megapixels (MP)
* Micro-USB 3.0 connection port
* 35 grams

## The camera mount

We originally planned to hold the oCams steady by CADing and 3D printing two mounts to hold them snugly. We then fixed the mounts in place relative to each other by bolting them to a piece of sheet metal cut to size.

![Camera Mount](/hiro/uploads/oCam_Mount.png "Camera Mount")

The bolts weren't sufficient to keep the mounts from shifting minutely, though, and this resulted in offsets in camera calibration. In the end, we merged the two camera mounts into one unified, 3D printed mount.

This was a better solution, as the single mount was structurally stiffer than two constrained single camera mounts.

The mount was held in place inside the head through 4 screw "pins," (2 on either side of the head). The pin forces exerted by the screws were sufficient to hold the mount immobile in the head under stress.

## Future improvements

### An additional clip on the back of the camera mount (to secure the cable to the mount)

While our mount design was adequate for holding our cameras steady, the micro-USB 3.0 to USB 2.0 cables had a tendency to come unplugged from the cameras when handled or disturbed too much. We resoldered the micro-USB 3.0 female ports on the oCams, which reduced instances of this problem, but did not completely solve them.

### Better (and cheaper) cameras

Having used the oCams for a semester, we have identified the important factors when choosing a camera for visual processing.

Of these factors, the most relevant to us are:

1. The shutter (global v. rolling)
2. Camera synchronization
3. Field of view (Wide v. narrow)
4. Color v. Black and White

**Global shutter:** Since our cameras are mounted inside Edwin's head, which moves fairly often as he executes behaviors, we need to be able to capture pixels in our images instantaneously, rather than sequentially. This will allow us to avoid image blurs and distortions from the motion of the cameras, which could potentially interfere with our visual processing.

**Camera synchronization:** If we could synchronize video feeds from both cameras, we would eliminate a potential source of error in our visual processing.

**Wide FOV:** If we can provide more data for our visual processing, we can cope with missing information more easily, thus creating a more accurate distance estimate overall.

**Color:** While most visual processing only requires black and white camera feeds, we also account for the dominant color of an object, which requires a color camera.