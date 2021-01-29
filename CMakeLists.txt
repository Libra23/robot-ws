cmake_minimum_required(VERSION 3.5)

#include cmake
if(DEFINED ENV{IDF_PATH})
  message("!!! ESP_PLATFORM !!!")
  set(EXTRA_COMPONENT_DIRS "./src")
  include($ENV{IDF_PATH}/tools/cmake/project.cmake)
endif()

# set the project name and version
project(robot-ws CXX)

if(NOT DEFINED ENV{IDF_PATH})
  message("!!! Linux !!!")
  # option
  set(CMAKE_CXX_FLAGS "-O2 -std=c++11 -Wall")
  # sub directory
  add_subdirectory(src)
  add_subdirectory(test)
endif()