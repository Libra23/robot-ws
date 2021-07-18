# src
message("!!! src !!!")

if(${ESP_PLATFORM})
    message("!!! ESP_PLATFORM !!!")
    #create object
    idf_component_register(SRCS "app_main.cpp" INCLUDE_DIRS ".")
    # subdirectory
    add_subdirectory(algorithm)
    add_subdirectory(control_data)
    add_subdirectory(common)
    add_subdirectory(robot)
    add_subdirectory(io_interface)
    add_subdirectory(kinematics)
    # register library
    target_link_libraries(${COMPONENT_LIB} algorithm common robot io_interface kinematics)
else()
    # create excute object
    add_executable(main.out main.cpp)
    # subdirectory
    add_subdirectory(algorithm)
    add_subdirectory(kinematics)
    # register library
    target_link_libraries(main.out algorithm kinematics)
endif()