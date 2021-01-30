# src
message("!!! src !!!")

if(${ESP_PLATFORM})
    message("!!! ESP_PLATFORM !!!")
    #create object
    idf_component_register(SRCS "app_main.cpp" INCLUDE_DIRS ".")
    # subdirectory
    add_subdirectory(algorithm)
    # register library
    target_link_libraries(${COMPONENT_LIB} algorithm)
else()    
    # create excute object
    add_executable(main.out main.cpp)
    # subdirectory
    add_subdirectory(algorithm)
    # register library
    target_link_libraries(main.out algorithm)
endif()