# src/control
message("!!! src/kinematic !!!")

# make static library
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRCS)
add_library(kinematics STATIC ${SRCS})

# link library
target_link_libraries(kinematics algorithm)
target_include_directories(kinematics PUBLIC ${CMAKE_SOURCE_DIR}/src)