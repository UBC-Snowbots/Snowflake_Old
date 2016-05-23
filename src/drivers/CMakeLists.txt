cmake_minimum_required(VERSION 2.8.3)
project(drivers)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    laser_geometry
    message_generation
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf
    messages
    tf2)
add_message_files(
  FILES
  gps.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

include(Catch)

add_definitions(--std=c++11)

catkin_package(
  CATKIN_DEPENDS std_msgs messages roscpp tf geometry_msgs message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ./include
)

add_library(sb_serial_communication src/SerialCommunication.cpp)

add_executable(sb_driver_elsa src/elsa_driver.cpp)

add_dependencies(sb_driver_elsa sb_msgs_generate_messages_cpp)

target_link_libraries(sb_driver_elsa sb_serial_communication ${catkin_LIBRARIES})


# add_executable(sb_sensor_driver src/sensor_driver.cpp)
# add_dependencies(sb_sensor_driver sb_msgs_generate_messages_cpp)
# target_link_libraries(sb_sensor_driver sb_serial_communication ${catkin_LIBRARIES})

add_executable(sb_gps_driver src/gps_driver.cpp)
add_dependencies(sb_gps_driver messages sb_msgs_generate_messages_cpp)
target_link_libraries(sb_gps_driver sb_serial_communication ${catkin_LIBRARIES})
