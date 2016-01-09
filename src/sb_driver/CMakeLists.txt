cmake_minimum_required(VERSION 2.8.3)
project(sb_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sb_msgs
  message_runtime
  tf
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs sb_msgs message_runtime
)


 add_library(sb_serial_communication
   src/SerialCommunication.cpp
 )


 add_library(sb_driver_elsa_headers src/avalanche_driver.cpp)
 add_library(sb_driver_avalanche_headers src/elsa_driver.cpp)
 add_executable(sb_driver_avalanche src/avalanche_driver.cpp)
 add_executable(sb_driver_elsa src/elsa_driver.cpp)
 add_dependencies(sb_driver_avalanche sb_msgs_generate_messages_cpp)
 add_dependencies(sb_driver_elsa sb_msgs_generate_messages_cpp)
 add_dependencies(sb_driver_avalanche_headers sb_msgs_generate_messages_cpp)
 add_dependencies(sb_driver_elsa_headers sb_msgs_generate_messages_cpp)

 target_link_libraries(
  sb_driver_avalanche
  sb_serial_communication
  sb_driver_avalanche_headers
  ${catkin_LIBRARIES}
 )


target_link_libraries(
  sb_driver_elsa
  sb_serial_communication
  sb_driver_elsa_headers
  ${catkin_LIBRARIES}
)
