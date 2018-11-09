---
title: Edwin Installation Instructions
layout: default
date: 2018-01-02
excerpt: "Instructions to install necessary software for running Edwin"
thumbnail:
categories: [Archives]
published: false
---

### Install Ubuntu 16.04
1. Create an empty partition with Windows Disk Management
2. Find a bootable USB drive with Ubuntu 16.04
3. Plug in the USB drive and restart your computer. When the Dell icon shows up, press F12 key until the "one-time bootup" screen shows up. 
4. Click on the "Install Ubuntu" option
5. Follow the on-screen prompts until the installation is complete. Restart your computer and your Ubuntu should be ready to use.
(Note: If your computer already has Ubuntu 14.04 or other previous versions of Ubuntu, you can select "Upgrade to 16.04" in Step 5. In this way, you can keep your personal document and installed packages.)

### Install ROS kinetic
To install ROS kinetic on your Ubuntu system, follow the instruction on the site below:
http://wiki.ros.org/kinetic/Installation/Ubuntu

### Getting Code to Run
1. Clone [github repository](https://github.com/olinrobotics/hiro) into your `catkin_ws/src` folder
2. Install dependencies:

  Basic processes:

    - copy
    - random
    - time
    - logging
    - datetime
    - pickle
    - os
    - sys
    - serial
    - re
    - shlex
    - threading
    - subprocess
    - struct

  Ros packages

    - std_msgs
    - common_msgs (sensor_msgs)
    - rospy
    - rospkg
    - roslib

  Mathematical processing:

    - math
    - numpy
    - scipy
    - operator
    - itertools

  Visual processing:

    - cv2
    - cv_bridge
    - Tkinter
    - csv

  Audial processing:

    - pyaudio
    - Queue
    - alsaaudio
    - audioop
    - wave
    - pocketsphinx
    - sphinxbase
    - espeak

3. *Under construction*

### Launch Files
[**`robot.launch`**](https://github.com/olinrobotics/edwin/blob/master/launch/robot.launch)
Fully operational launch of all demo systems.

Requirements:

  - Loads Edwin's brain (brain.py)
  - Loads idle behaviors (idle.py)
  - USB cam feed required (image_raw, camera_info)
  - Arm must be on (arm_node.py, arm_behaviors.py, arm_draw.py)

[**`robot_minimal.launch`**](https://github.com/olinrobotics/edwin/blob/master/launch/robot_minimal.launch)
Mostly for quick behavior debugging. This is the minimum needed to "operate" Edwin. No camera.

Requirements:

  - Loads Edwin's brain (brain.py)
  - Arm must be on (arm_behaviors.py)
  - Kinect must be on and connected.

[**`robot_sight.launch`**](https://github.com/olinrobotics/edwin/blob/master/launch/robot_sight.launch)
Meant for testing only. This launch file only loads camera and Kinect scripts; the arm does not boot up.

Requirements:

  - Kinect must be on and connected.
  - USB cam feed required (image_raw, camera_info).
