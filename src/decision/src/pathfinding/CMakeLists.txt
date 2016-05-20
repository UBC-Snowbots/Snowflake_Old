add_executable(pathfinding pathfinding.cpp)
add_executable(pub_map pub_test_occgrid.cpp)
add_executable(pub_init_point pub_test_poseinit.cpp)
add_executable(pub_final_point pub_test_posefinal.cpp)
add_executable(sub_test_path sub_test_path.cpp)


target_link_libraries(pathfinding ${catkin_LIBRARIES})
target_link_libraries(pub_map ${catkin_LIBRARIES})
target_link_libraries(pub_init_point ${catkin_LIBRARIES})
target_link_libraries(pub_final_point ${catkin_LIBRARIES})
target_link_libraries(sub_test_path ${catkin_LIBRARIES})