#include "io_interface.hpp"
#include "esp_log.h"
/**
 * @brief Global parameter
 */
ShareMemory<OutputState> output_memory_;
ShareMemory<InputState> input_memory_;

/**
 * @class IoInterface
 */
IoInterface::IoInterface() {
    ESP_LOGI("Io Interface", "Constructor");
}

void IoInterface::Thread() {
    ESP_LOGI("Io Interface", "Thread");

    // set config
    CreateConfig(config_);
    serial_servo_.Config(config_.serial_servo);

    // main loop
    while(true) {
        OutputState output_state_out;
        //ESP_LOGI("Io Interface", "en_pin = %d", config_.serial_servo.en_pin);
        output_memory_.Read(output_state_out);
        UpdateOutput(output_state_out);
        delay(10);
    }
}

void IoInterface::CreateConfig(IoInterfaceConfig& config) {
    // serial servo
    config.serial_servo.port_id = 1;
    config.serial_servo.tx_pin = 23;
    config.serial_servo.rx_pin = 19;
    config.serial_servo.en_pin = 33;
    config.serial_servo.baud_rate = 115200;
    config.serial_servo.time_out_ms = 50;
}

void IoInterface::UpdateInput(InputState& state) {
    // input serial servo
    for(auto& serial_servo_state : state.serial_servo) {
        if (serial_servo_state.id < 0) {
            return;
        } else if (serial_servo_state.enable) {
            serial_servo_state.act_q = serial_servo_.GetPosition(serial_servo_state.id);
        } else {
            serial_servo_state.act_q = serial_servo_.FreePosition(serial_servo_state.id);
        }
    }
}

void IoInterface::UpdateOutput(const OutputState& state) {
    // output serial servo
    for(const auto& serial_servo_state : state.serial_servo) {
        //ESP_LOGI("Io Interface", "id = %d, q = %f", serial_servo_state.id, serial_servo_state.act_qq);
        if (serial_servo_state.id < 0) {
            return;
        } else if (serial_servo_state.enable) {
            serial_servo_.SetPosition(serial_servo_state.id, serial_servo_state.act_q);
        } else {
            serial_servo_.FreePosition(serial_servo_state.id);
        }
    }
}

/**
 * @class IoInterfaceMain
 */
IoInterfaceMain::IoInterfaceMain() {
    ESP_LOGI("Io Interface Main", "Constructor");
}

void IoInterfaceMain::Run() {
    ESP_LOGI("Io Interface Main", "Run");
    th_.Start(IoInterfaceMain::LaunchThread, "io_thread", 5, 4096, &io_interface_, 1);
}

void IoInterfaceMain::LaunchThread(void* arg) {
    ESP_LOGI("Io Interface Main", "Launch");
    reinterpret_cast<IoInterface*>(arg)->Thread();
}