# test/algorithm
message("!!! test/algorithm !!!")

# create excute object
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRCS)
add_executable(algorithm_test ${SRCS})

# link library
target_link_libraries(algorithm_test algorithm gtest_main)
target_include_directories(algorithm_test PRIVATE ${CMAKE_SOURCE_DIR}/src/algorithm)

find_package(Python3 COMPONENTS Development NumPy)
target_include_directories(algorithm_test PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
target_link_libraries(algorithm_test Python3::Python Python3::NumPy)

# make test
add_test(algorithm_test algorithm_test)