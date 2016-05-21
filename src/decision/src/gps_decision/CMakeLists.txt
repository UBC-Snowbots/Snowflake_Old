add_executable(gps_decision gps_decision.cpp)
add_dependencies(gps_decision sb_messages)
target_link_libraries(gps_decision ${catkin_LIBRARIES})

add_executable(gps_decision_test gps_decision_test.cpp)
add_dependencies(gps_decision_test sb_messages)
target_link_libraries(gps_decision_test ${catkin_LIBRARIES})
