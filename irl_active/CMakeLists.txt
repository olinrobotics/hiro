cmake_minimum_required(VERSION 2.8.3)
project(irl)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	roslib
	std_msgs
	message_generation
##	rosserial_arduino
##	rosserial_client
	cv_bridge
	image_transport
	geometry_msgs
	sensor_msgs
	tf
)

# Find OpenNI
find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)

#Find OpenGL
find_package(OpenGL REQUIRED)

#Find GLUT
find_package(GLUT REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY
	msg
	FILES
	Edwin_Shape.msg
	pointerpos.msg
	edwin_gestures.msg
	People.msg
	SceneAnalysis.msg
	Bones.msg
	HHH.msg
	blue_button.msg
	yellow_button.msg
	red_button.msg
	green_button.msg
	all_buttons.msg
	minimap.msg
	blocks.msg
	Real_Cube.msg
	Grid_Cube.msg
	Real_Structure.msg
	Grid_Structure.msg
	Cube_Structures.msg

)
add_service_files(
	DIRECTORY
	srv
	FILES
	AddTwoInts.srv
	arm_cmd.srv
)
##Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)


catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs)



install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
	PATTERN ".svn" EXCLUDE)

install(FILES package.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
