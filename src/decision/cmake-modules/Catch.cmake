_generate_function_if_testing_is_disabled("catkin_add_gtest")

set(MODULE_DIR ${CMAKE_CURRENT_LIST_DIR})

function(add_catch_test target)
	catkin_run_tests_target("catch" ${target} "catch-${target}.xml"
		COMMAND
			"python ${MODULE_DIR}/run_catch_test.py --test $<TARGET_FILE:${target}> --post_proc ${MODULE_DIR}/strip_junit.py --output ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/catch-${target}.xml"
		DEPENDENCIES ${target} WORKING_DIRECTORY $<TARGET_FILE_DIR:${target}>)
endfunction(add_catch_test)
