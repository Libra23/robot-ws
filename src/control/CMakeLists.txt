# src/control
message("!!! src/control !!!")

# make static library
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRCS)
add_library(control STATIC ${SRCS})

# link library
target_link_libraries(control algorithm kinematics)
target_include_directories(control PUBLIC ${CMAKE_SOURCE_DIR}/src)