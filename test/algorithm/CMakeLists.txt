# test/algorithm
message("!!! test/algorithm !!!")

# create excute object
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRCS)
add_executable(algorithm_test ${SRCS})

# link library
target_link_libraries(algorithm_test algorithm gtest_main)
target_include_directories(algorithm_test PRIVATE ${CMAKE_SOURCE_DIR}/src/algorithm)

find_package(PythonLibs 2.7)
target_link_libraries(algorithm_test ${PYTHON_LIBRARIES})
target_include_directories(algorithm_test PRIVATE ${PYTHON_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/test/matplotlib)


# make test
add_test(algorithm_test algorithm_test)