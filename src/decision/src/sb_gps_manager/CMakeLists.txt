add_executable(gps_manager sb_gps.cpp)
add_dependencies(gps_manager sb_msgs_generate_messages_cpp)
target_link_libraries(gps_manager ${catkin_LIBRARIES})
