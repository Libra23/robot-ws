# src

# library
## eigen
include_directories(${EIGEN3_INCLUDE_DIR})

# subdirectory
add_subdirectory(eigen)
add_subdirectory(utility)

# create excute object
add_executable(main.out main.cpp)

# register library
target_link_libraries(main.out Eigen3::Eigen)