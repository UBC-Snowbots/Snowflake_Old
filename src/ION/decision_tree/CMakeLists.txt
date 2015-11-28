cmake_minimum_required (VERSION 2.6)
project (decision_tree)
add_executable(decision_tree main.cpp decision_tree.cpp decision_tree.hpp catch.hpp)
add_executable(decision_tree_test test.cpp decision_tree.cpp decision_tree.hpp catch.hpp)

