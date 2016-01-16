
add_library(decision_tree decision_tree.cpp decision_tree.hpp)

add_executable(decision_tree_main main.cpp)
target_link_libraries(decision_tree_main decision_tree)

add_executable(decision_tree_test test.cpp)
target_link_libraries(decision_tree_test decision_tree)

add_catch_test(decision_tree_test)

# Copy testing files over so the built tests can access them
configure_file(map2_width_10.map $<TARGET_FILE_DIR:decision_tree_test>/map2_width_10.map COPYONLY)
configure_file(map3_width_10.map $<TARGET_FILE_DIR:decision_tree_test>/map3_width_10.map COPYONLY)
