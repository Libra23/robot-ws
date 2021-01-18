# src/utility

# make library
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRCS)
add_library(utility ${SRCS})

# link library with others
target_link_libraries(utility eigen)