## Edwin\_Stereo

This package is specifically for stereoscopic vision for [Edwin](http://github.com/olinrobotics/edwin), a Eusocial Coworker Robot Project for Olin Robotics Lab.

## Building

This package has several dependencies that are not part of the standard ROS Indigo Ecosystem.

- OpenCV 3 (3.2.0)

## RUNNING THE DEMO

1. Run roscore:

	```bash
	roscore
	```
2. Run Hardware Interface with ST-R17 arm:

	```bash
	roslaunch edwin_moveit_config hardware.launch
	```

	Edwin should start calibrating after launch.

3. Run rosserial:

	```bash
	roslaunch edwin_moveit_config rosserial.launch
	```

	To operate the gripper, ensure that the DC power supply is on.

	Try publishing to /grip topic, in a range from 55~70, to test the gripper.

4. Run MoveIt! RViz Plugin:

	```bash
	roslaunch edwin_moveit_config real.launch
	```

	This will help visualize its current status.

5. Run Move\_Group\_Interface:

	```bash
	roslaunch edwin_moveit_config move_group_interface.launch
	```

	This is the node that directly listens to motion commands.

6. Launch Vision and Speech-Related Code:

	```bash
	roslaunch edwin_stereo demo.launch
	```

7. [Optional] For debugging purposes, run rqt\_reconfigure:

	```bash
	#rosrun rqt_reconfigure rqt_reconfigure
	```
