add_subdirectory(decision_tree)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(moveStraightLine moveStraightLine.cpp)
target_link_libraries(moveStraightLine ${catkin_LIBRARIES})

