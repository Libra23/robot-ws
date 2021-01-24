# src/algorithm

# make library
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRCS)
add_library(algorithm STATIC  ${SRCS})

# include library
target_include_directories(algorithm PUBLIC ${CMAKE_SOURCE_DIR}/src/eigen)