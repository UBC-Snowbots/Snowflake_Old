add_executable(pathfinding pathfinding.cpp)
add_executable(pub_map pub_test_occgrid.cpp)
add_executable(pub_init_point pub_test_poseinit.cpp)
add_executable(pub_final_point pub_test_posefinal.cpp)


target_link_libraries(pathfinding ${catkin_LIBRARIES})
target_link_libraries(pub_map ${catkin_LIBRARIES})
target_link_libraries(pub_init_point ${catkin_LIBRARIES})
target_link_libraries(pub_final_point ${catkin_LIBRARIES})