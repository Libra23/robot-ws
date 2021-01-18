# prohibit cmake in top working directory
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "DO NOT BUILD in-tree.")
endif()

cmake_minimum_required(VERSION 3.1)

# set the project name and version
project(robot-ws CXX)

set(CMAKE_CXX_FLAGS "-O2 -std=c++11 -Wall")

add_subdirectory(src)