# Human Interactions Robotics Laboratory
UR5 interactive co-working twin robotic arms, Castor and Pollux. Check out [our Wiki](https://github.com/olinrobotics/hiro/wiki) for current documentation.

Website: [olinrobotics.github.io/hiro/](https://olinrobotics.github.io/hiro/)

UR5/CB3 Arm Manual: https://www.usna.edu/Users/weapron/kutzer/_files/documents/User%20Manual,%20UR5.pdf

## Quick Setup
- Install [ROS](http://wiki.ros.org/)
- Go to your catkin workspace's source folder: `cd <your_catkin_ws>/src`
- Clone this project: `git clone https://github.com/olinrobotics/hiro.git`
- Go back one level to catkin workspace: `cd ..`
- Install dependencies: `rosdep install --from-paths src --ignore-src -r -y`
- Build the platform: `catkin_make`

## Team Repositories
- [olinrobotics/hiro](/olinrobotics/hiro): Main repository where we store our projects.
- [olinrobotics/universal_robot](/olinrobotics/universal_robot): repository that stores the URDFs of UR3/UR5/UR10 arms and the Xamyab robot. It also contains the simulated Gazebo setups and the MoveIt! configurations for these models. Note that it does not have the gripper's URDF model (next repo).
- [olinrobotics/robotiq_2finger_grippers](/olinrobotics/robotiq_2finger_grippers): contains the URDF for the 2f_140 gripper. It also has the controller that works with the real gripper. 
- [olinrobotics/ur_modern_driver](/olinrobotics/ur_modern_driver): is used to connect with the real UR arms. It contains the launch file to run the real robot.
- [olinrobotics/iai_kinect2](/olinrobotics/iai_kinect2): ROS package for Xbox Kinect camera.

## Troubleshooting

Grouped by error message

*Encoder-stepper mismatch*
* Is the area around the K11R control box and robot arm clear?
* Turn the controller on/off. (Power-cycle the robot)
* The arm generally makes a pseudo circle using its waist at the beginning of the calibration, done counterclockwise. Try starting it closer to its end point for that rotation, and then execute the startup code. The same thing could be tried for any of the joints (or axes, which is what Roboforth refers to them by).

---

*Controller refuses to turn on*
* Check the fuses. Two are located in the back of the controller, one is located on the power supply itself. (Where the power cord is plugged in)
