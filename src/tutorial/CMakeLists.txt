cmake_minimum_required(VERSION 2.8.3)
project(tutorial)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs rospy)
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
catkin_install_python(PROGRAMS scripts/py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
