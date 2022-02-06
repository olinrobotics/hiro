# install 
source /opt/ros/<your_ros_version>/setup.bash
git clone -b boost https://github.com/UniversalRobots/Universal_Robots_Client_Library.git src/third-party/Universal_Robots_Client_Library
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/third-party/Universal_Robots_ROS_Driver
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/third-party/fmauch_universal_robot
git clone https://github.com/Danfoa/robotiq_2finger_grippers src/third-party/robotiq_2finger_grippers
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y