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
        OutputState output;
        output_memory_.Read(output);
        UpdateOutput(output);
        delay(10);
    }
}

void IoInterface::CreateConfig(IoInterfaceConfig& config) {
    // serial servo
    config.serial_servo.port_id = 1;
    config.serial_servo.tx_pin = 23;
    config.serial_servo.rx_pin = 19;
    config.serial_servo.en_pin = 33;
    config.serial_servo.baud_rate = 1250000;
    config.serial_servo.time_out_ms = 10;
}

void IoInterface::UpdateInput(InputState& state) {
    // input serial servo
    for(auto& serial_servo_state : state.serial_servo) {
        if (serial_servo_state.id < 0) {
            return;
        } else {
            serial_servo_state.act_q = serial_servo_.GetPosition(serial_servo_state.id);
            // serial_servo_state.act_q = serial_servo_.FreePosition(serial_servo_state.id);
        }
    }
}

void IoInterface::UpdateOutput(const OutputState& state) {
    // output serial servo
    for(const auto& serial_servo_state : state.serial_servo) {
        if (serial_servo_state.id < 0) {
            return;
        } else if (serial_servo_state.enable) {
            double act_q = serial_servo_.SetPosition(serial_servo_state.id, serial_servo_state.act_q);
            // ESP_LOGI("Io Interface", "id = %d, act_q = %f", serial_servo_state.id, act_q);
        } else {
            double act_q = serial_servo_.FreePosition(serial_servo_state.id);
            // ESP_LOGI("Io Interface", "id = %d, act_q = %f", serial_servo_state.id, act_q);
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
    th_.Start(IoInterfaceMain::LaunchThread, "io_thread", 1, 4096, &io_interface_, 1);
}

uint32_t IoInterfaceMain::StackMargin() {
    return th_.GetStackMargin();
}

void IoInterfaceMain::LaunchThread(void* arg) {
    ESP_LOGI("Io Interface Main", "Launch");
    reinterpret_cast<IoInterface*>(arg)->Thread();
}