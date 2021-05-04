# test/algorithm
message("!!! test/kinematics !!!")

# create excute object
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRCS)
add_executable(kinematics_test ${SRCS})

# link library
target_link_libraries(kinematics_test kinematics gtest_main)
target_include_directories(kinematics_test PRIVATE ${CMAKE_SOURCE_DIR}/src/kinematics)

find_package(PythonLibs 2.7)
target_link_libraries(kinematics_test ${PYTHON_LIBRARIES})
target_include_directories(kinematics_test PRIVATE ${PYTHON_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/test/matplotlib)


# make test
add_test(kinematics_test kinematics_test)