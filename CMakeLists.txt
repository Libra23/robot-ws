cmake_minimum_required(VERSION 3.5)

#include cmake
if(ESP_PLATFORM)
  message("!!! ESP_PLATFORM !!!")
  set(EXTRA_COMPONENT_DIRS "./src")
  include($ENV{IDF_PATH}/tools/cmake/project.cmake)
endif()

# set the project name and version
project(robot-ws)

if(NOT ESP_PLATFORM)
  message("!!! CXX !!!")
  # option
  set(CMAKE_CXX_FLAGS "-O2 -std=c++11 -Wall")
  # sub directory
  add_subdirectory(src)
  add_subdirectory(test)
endif()