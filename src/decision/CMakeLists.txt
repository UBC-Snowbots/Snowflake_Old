cmake_minimum_required(VERSION 2.8.3)
project(decision)

add_service_files(
  FILES
  gps_waypoint.srv
)
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
generate_messages(DEPENDENCIES 
std_msgs
)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

include(Catch)

add_definitions(--std=c++11)

catkin_package(
  CATKIN_DEPENDS std_msgs roscpp tf geometry_msgs message_runtime
)

find_package(Armadillo REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
    ./include
)

add_subdirectory(src)

# Testing Stuff
add_executable(MoveStraightLineTest test/MoveStraightLineTest.cpp test/main.cpp)
target_link_libraries(MoveStraightLineTest libmove_straight_line)
add_catch_test(MoveStraightLineTest)



