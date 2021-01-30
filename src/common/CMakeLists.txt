# src/io_interface
message("!!! src/common !!!")

# make library
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRCS)
add_library(common STATIC ${SRCS})