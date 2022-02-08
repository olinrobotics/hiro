# install 
source /opt/ros/<your_ros_version>/setup.bash
rm -rf src/third-party/
git clone -b boost https://github.com/UniversalRobots/Universal_Robots_Client_Library.git src/third-party/Universal_Robots_Client_Library
git clone https://github.com/olinrobotics/Universal_Robots_ROS_Driver.git src/third-party/Universal_Robots_ROS_Driver
git clone -b https://github.com/olinrobotics/universal_robot.git src/third-party/universal_robot
rm -rf src/third-party/universal_robot/ur_msgs
git clone https://github.com/olinrobotics/robotiq_2finger_grippers.git src/third-party/robotiq_2finger_grippers
git clone https://github.com/ros-industrial/ur_msgs.git src/third-party/ur_msgs
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y