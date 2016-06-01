cmake_minimum_required(VERSION 2.8.3)

project(opencv)

add_definitions(--std=c++11)

## Find catkin macros and libraries

## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)

## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS tf laser_geometry roscpp rospy cv_bridge genmsg image_transport sensor_msgs)

find_package(OpenCV REQUIRED)

find_package(PCL 1.3 REQUIRED)


include_directories(

  ${catkin_INCLUDE_DIRS}

  ${OpenCV_INCLUDE_DIRS}

  ${PCL_INCLUDE_DIRS}

)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})



catkin_package()

add_executable(ros_vision src/ros_vision.cpp src/filter.h src/filter.cpp src/IPM.h src/IPM.cpp)

add_executable(image_to_cloud src/imageToCloud.cpp src/filter.h src/filter.cpp src/IPM.h src/IPM.cpp)

add_executable(point_cloud_merger src/pointCloudMerger.cpp)

add_executable(laserscan_to_pointcloud src/laserscanToPointCloud.cpp)

add_executable(stitcher src/image_stitcher.cpp)

target_link_libraries(ros_vision ${catkin_LIBRARIES} ${OpenCV_LIBS})

target_link_libraries(image_to_cloud ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

target_link_libraries(point_cloud_merger ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

target_link_libraries(laserscan_to_pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

target_link_libraries(stitcher ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(ros_vision opencv)

add_dependencies(stitcher opencv)

add_dependencies(image_to_cloud opencv)

add_dependencies(laserscan_to_pointcloud laser_geometry)

