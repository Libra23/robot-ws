# src

# subdirectory
add_subdirectory(eigen)
add_subdirectory(utility)

# create excute object
add_executable(main.out main.cpp)

# register library
target_link_libraries(main.out utility)