
add_library(libmove_straight_line MoveStraightLine.cpp)
target_link_libraries(libmove_straight_line ${ARMADILLO_LIBRARIES})

add_executable(move_straight_line move_straight_line.cpp)
target_link_libraries(move_straight_line ${catkin_LIBRARIES} libmove_straight_line)
