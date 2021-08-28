# src/io_interface
message("!!! src/mainte !!!")

# make library
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRCS)
add_library(mainte STATIC ${SRCS})

# link library
target_link_libraries(mainte algorithm common idf::nvs_flash)
target_include_directories(mainte PUBLIC ${CMAKE_SOURCE_DIR}/src)