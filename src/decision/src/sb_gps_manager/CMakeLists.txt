add_executable(gps_decision gps_decision.cpp)
add_dependencies(gps_decision sb_messages)
target_link_libraries(gps_decision ${catkin_LIBRARIES})
