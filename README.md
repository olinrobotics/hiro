# Interactive Robotics Laboratory
ST-R17 and UR5 interactive co-working dinosaur (and dragon) robotic arms
Website: https://olinrobotics.github.io/irl/
Arm Manual: http://strobotics.com/manuals/R17%20manual.pdf

## Troubleshooting

Grouped by error message

*Encoder-stepper mismatch*
* Is the area around the K11R control box and robot arm clear?
* Turn the controller on/off. (Power-cycle the robot)
* The arm generally makes a pseudo circle using its waist at the beginning of the calibration, done counterclockwise. Try starting it closer to its end point for that rotation, and then execute the startup code. The same thing could be tried for any of the joints (or axes, which is what Roboforth refers to them by).

---

*rosrun irl arm_node.py is stuck at "in block_on_result"*
* Is the turn-key at the front of the controller set to warm?
* Is the light on the Tripp-Lite serial-usb converter blinking?
* Open a new terminal window and run the command again

---

*Controller refuses to turn on*
* Check the fuses. Two are located in the back of the controller, one is located on the power supply itself. (Where the power cord is plugged in)
