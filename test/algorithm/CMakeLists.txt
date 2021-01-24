# test/algorithm

# create excute object
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRCS)
add_executable(algorithm_test ${SRCS})

# link library
target_link_libraries(algorithm_test algorithm gtest_main)
target_include_directories(algorithm_test PUBLIC ${PROJECT_SOURCE_DIR}/src/algorithm)

# make test
add_test(algorithm_test algorithm_test)