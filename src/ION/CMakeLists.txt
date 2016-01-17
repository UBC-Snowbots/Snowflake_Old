cmake_minimum_required(VERSION 2.8.3)
project(ION)

find_package(catkin REQUIRED COMPONENTS 
    geometry_msgs
    laser_geometry
    message_generation
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

include(Catch)

add_subdirectory(msg)
add_subdirectory(srv)

add_definitions(--std=c++11)

catkin_package(
  CATKIN_DEPENDS std_msgs roscpp tf geometry_msgs
)

find_package(Armadillo REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${laser_geometry_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
    ./include
)

add_subdirectory(src)
add_subdirectory(test)
