add_library(libpathfinder Pathfinder.cpp)
target_link_libraries(libpathfinder ${catkin_LIBRARIES})

add_executable(astar AStar.cpp)
target_link_libraries(astar ${catkin_LIBRARIES} libpathfinder)
