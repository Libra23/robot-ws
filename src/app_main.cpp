#include "robot/robot_control.hpp"
#include "io_interface/io_interface.hpp"
#include "mainte/mainte.hpp"
#include <iostream>
/**
 * @brief main function
 */
extern "C" void app_main () {
    // prepare for log
    delay(10000);

    // declare thread
    RobotMain robot_main;
    IoInterfaceMain io_interface_main;
    MaintenanceMain mainte_main;

    // run thread
    robot_main.Run();
    io_interface_main.Run();
    mainte_main.Run();

    // run threads monitor
    while(true) {
        std::cout << "robot_thread_margin = " << robot_main.StackMargin() << std::endl;
        std::cout << "io_thread_margin = " << io_interface_main.StackMargin() << std::endl;
        std::cout << "mainte_thread_margin = " << mainte_main.StackMargin() << std::endl;
        delay(10000);
    }
}