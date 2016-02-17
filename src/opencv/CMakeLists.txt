cmake_minimum_required(VERSION 2.8.3)

project(opencv)



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




add_executable(color_analyzer src/color_analyzer.cpp)

add_executable(ipm src/main.cpp src/IPM.h src/IPM.cpp)

add_executable(mouse src/mouse.cpp)

target_link_libraries(color_analyzer ${catkin_LIBRARIES} ${OpenCV_LIBS})

target_link_libraries(ipm ${catkin_LIBRARIES} ${OpenCV_LIBS})

target_link_libraries(mouse ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(color_analyzer opencv)

add_dependencies(ipm opencv)

add_dependencies(mouse opencv)
