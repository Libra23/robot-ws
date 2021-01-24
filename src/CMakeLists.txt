# src

# subdirectory
add_subdirectory(algorithm)
add_subdirectory(gtest)

# create excute object
add_executable(main.out main.cpp)

# register library
target_link_libraries(main.out algorithm)