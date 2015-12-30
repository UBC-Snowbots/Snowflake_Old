cmake_minimum_required(VERSION 2.8.3)
project(opencvshenanigans)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(imageload src/imageload.cpp)
add_executable(grays src/grays.cpp)
add_executable(webcam src/webcam.cpp)
add_executable(optical_flow src/optical_flow.cpp)
target_link_libraries(imageload  ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(grays ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(webcam ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(optical_flow ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(imageload opencvshenanigans)
add_dependencies(grays opencvshenanigans)
add_dependencies(webcam opencvshenanigans)
add_dependencies(optical_flow opencvshenanigans)
