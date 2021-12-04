#include "robot/robot_control.hpp"
#include "io_interface/io_interface.hpp"
#include "mainte/mainte.hpp"
#include <iostream>
#include "common/queue.hpp"
#include "control_data/msg_data.hpp"
#include "esp_log.h"
/**
 * @brief main function
 */
extern "C" void app_main () {
    // prepare for log
    delay(10000);

    // declare thread
    //RobotMain robot_main;
    //IoInterfaceMain io_interface_main;
    //MaintenanceMain mainte_main;

    // run thread
    //robot_main.Run();
    //io_interface_main.Run();
    //mainte_main.Run();
    Queue queue(10, GetMaxMsgSize());

    MsgCmd cmd(MAINTE_TO_ROBOT_CONTROL_OFF);
    queue.Send(&cmd);

    MsgCmdControlOn cmd_control_on(3);
    queue.Send(&cmd_control_on);

    ESP_LOGI("Main", "Queue depth = %d, size = %d", queue.NumOfItems(), GetMaxMsgSize());
    uint32_t num_of_msg = queue.NumOfItems();
    for (uint32_t i = 0; i < num_of_msg; i++) {
        uint8_t buf[GetMaxMsgSize()];
        queue.Receive(buf);
        uint8_t type = GetMsgType(buf);

        switch(type) {
            case MAINTE_TO_ROBOT_CONTROL_OFF: {
                const MsgCmdByte res(buf);
                ESP_LOGI("Main", "Msg Type:MAINTE_TO_ROBOT_CONTROL_OFF, Queue size = %d, index = %d", queue.NumOfItems(), i);
                break;
            }

            case MAINTE_TO_ROBOT_CONTROL_ON: {
                const MsgCmdControlOn  res(buf);
                ESP_LOGI("Main", "Msg Type:MAINTE_TO_ROBOT_CONTROL_ON, arm_id = %d", res.arm_id);
                break;
            }
            default:
                ESP_LOGI("Main", "Msg Type = %d, Queue size = %d, index = %d", type, queue.NumOfItems(), i);
                break;
        }
    }

    // run threads monitor
    while(true) {
        //std::cout << "robot_thread_margin = " << robot_main.StackMargin() << std::endl;
        //std::cout << "io_thread_margin = " << io_interface_main.StackMargin() << std::endl;
        //std::cout << "mainte_thread_margin = " << mainte_main.StackMargin() << std::endl;
        delay(10000);
    }
}