cmake_minimum_required(VERSION 2.8.3)
project (sb_messages)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

add_message_files(
  FILES
  gps.msg
)

generate_messages(
  DEPENDENCIES
  sb_messages
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
