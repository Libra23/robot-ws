# test/algorithm
message("!!! test/kinematics !!!")

# create excute object
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRCS)
add_executable(kinematics_test ${SRCS})

# link library
target_link_libraries(kinematics_test kinematics gtest_main)
target_include_directories(kinematics_test PRIVATE ${CMAKE_SOURCE_DIR}/src/kinematics)

find_package(Python3 COMPONENTS Development NumPy)
target_link_libraries(kinematics_test Python3::Python Python3::NumPy)
target_include_directories(kinematics_test PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/test/matplotlib)

# make test
add_test(kinematics_test kinematics_test)