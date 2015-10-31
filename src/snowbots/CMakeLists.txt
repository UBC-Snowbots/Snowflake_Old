cmake_minimum_required(VERSION 2.8.3)
project(snowbots)
find_package(catkin REQUIRED COMPONENTS message_generation
	std_msgs rospy roscpp sensor_msgs
	laser_geometry tf2)

add_subdirectory(msg)
add_subdirectory(srv)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs roscpp
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${laser_geometry_INCLUDE_DIRS}
)

add_subdirectory(src)
