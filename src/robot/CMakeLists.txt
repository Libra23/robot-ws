# src/robot
message("!!! src/robot !!!")

# make static library
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRCS)
add_library(robot STATIC ${SRCS})

# link library
target_link_libraries(robot algorithm kinematics)
target_include_directories(robot PUBLIC ${CMAKE_SOURCE_DIR}/src)