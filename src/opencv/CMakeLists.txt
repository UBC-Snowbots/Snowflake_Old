cmake_minimum_required(VERSION 2.8.3)

project(opencv)

add_definitions(--std=c++11)

## Find catkin macros and libraries

## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)

## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS roscpp rospy cv_bridge genmsg image_transport sensor_msgs
)


find_package(OpenCV REQUIRED)



include_directories(

  ${catkin_INCLUDE_DIRS}

  ${OpenCV_INCLUDE_DIRS}

)


catkin_package()


add_executable(camera_publisher src/camera.cpp)

add_executable(camera_subscriber src/camera_subscriber.cpp)

add_executable(ros_vision src/ros_vision.cpp src/filter.h src/filter.cpp src/IPM.h src/IPM.cpp)

add_executable(image_to_laser_scan src/image_to_laser_scan.cpp)



target_link_libraries(camera_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS})

target_link_libraries(camera_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBS})

target_link_libraries(ros_vision ${catkin_LIBRARIES} ${OpenCV_LIBS})

target_link_libraries(image_to_laser_scan ${catkin_LIBRARIES})



add_dependencies(camera_publisher opencv)

add_dependencies(camera_subscriber opencv)

add_dependencies(ros_vision opencv)

