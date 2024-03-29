#include "robot/robot_control.hpp"
#include "io_interface/io_interface.hpp"
#include "mainte/mainte.hpp"
#include <iostream>

#include "esp_log.h"
/**
 * @brief main function
 */
extern "C" void app_main () {
    // prepare for log
    delay(3000);

    // declare thread
    IoInterfaceMain io_interface_main;
    RobotMain robot_main;
    MaintenanceMain mainte_main;

    // run thread
    io_interface_main.Run();
    robot_main.Run();
    mainte_main.Run();
 
    // run threads monitor
    while(true) {
        vTaskDelay(portMAX_DELAY);
        //std::cout << "robot_thread_margin = " << robot_main.StackMargin() << std::endl;
        //std::cout << "io_thread_margin = " << io_interface_main.StackMargin() << std::endl;
        //std::cout << "mainte_thread_margin = " << mainte_main.StackMargin() << std::endl;
    }
}