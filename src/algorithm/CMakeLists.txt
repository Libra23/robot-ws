# src/algorithm
message("!!! src/algorithm !!!")

# make static library
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRCS)
add_library(algorithm STATIC ${SRCS})

# link library
target_include_directories(algorithm PUBLIC ${CMAKE_SOURCE_DIR}/src/eigen)