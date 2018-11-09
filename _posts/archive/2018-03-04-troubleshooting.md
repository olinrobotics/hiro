---
title: Troubleshooting
layout: default
date: 2018-01-02
excerpt: "Troubleshooting"
categories: [Archives]
published: false
---


# Edwin Hardware details

ST-R17 co-worker dinosaur robotic arm
Website: https://olinrobotics.github.io/edwin/
Arm Manual: http://strobotics.com/manuals/R17%20manual.pdf
Head Module Design: 


# Troubleshooting

### Encoder-stepper mismatch

 Is the area around the K11R control box and robot arm clear?
 Turn the controller on/off. (Power-cycle the robot)

### rosrun edwin arm_node.py is stuck at "in block_on_result"

 Is the turn-key at the front of the controller set to warm?
 Is the light on the Tripp-Lite serial-usb converter blinking?
 Open a new terminal window and run the command again
 
### rosrun edwin arm_node.py returns "AttributeError: StArm instance hs no attribute 'cxn'"

  This is a permissions error.
  Type `groups <YOUR USERNAME>`
  Are you a member of "dialout"?
  If not, type `sudo adduser $(whoami) dialout`
  Type in your password.

### Controller refuses to turn on

 Check the fuses. Two are located in the back of the controller, one is located on the power supply itself. (Where the power cord is plugged in)
 
*This Github page is currently under construction. Last edited on 1/29/17 by [L. Zuehsow.](https://github.com/Oktober13)*
