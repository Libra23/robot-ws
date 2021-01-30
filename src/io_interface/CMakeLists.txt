# src/io_interface
message("!!! src/io_interface !!!")

# make library
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRCS)
add_library(io_interface STATIC ${SRCS})

# link library
target_link_libraries(io_interface common)
target_include_directories(io_interface PUBLIC ${CMAKE_SOURCE_DIR}/src)