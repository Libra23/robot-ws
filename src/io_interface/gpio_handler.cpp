#include "gpio_handler.hpp"
#include "esp_log.h"
/**
 * @class GpioHandler
 */
GpioHandler::GpioHandler() {
    ESP_LOGI("GpioHandler", "Constructor");
}

void GpioHandler::Config(const GpioConfig& config) {
    // ESP_LOGI("GpioHandler", "Config");
    const gpio_num_t pin = static_cast<gpio_num_t>(config.pin);
    gpio_reset_pin(pin);
    switch(config.dir) {
        case INPUT:
        gpio_set_direction(pin, GPIO_MODE_INPUT);
        break;
        case OUTPUT:
        if (config.open_drain) {
            gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
        } else {
            gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        }
        break;
        case INPUT_OUTPUT:
        if (config.open_drain) {
            gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT_OD);
        } else {
            gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT);
        }
        break;
    }
    
    if (config.pull_up) {
        gpio_pullup_en(pin);
    } else {
        gpio_pullup_dis(pin);
    }
    if (config.pull_down) {
        gpio_pulldown_en(pin);
    } else {
        gpio_pulldown_dis(pin);
    }
}

void GpioHandler::Write(uint8_t pin, uint8_t level) {
    // ESP_LOGI("GpioHandler", "Write pin = %d level = %d", pin, level);
    gpio_set_level(static_cast<gpio_num_t>(pin), level);
}

uint8_t GpioHandler::Read(uint8_t pin) {
    // ESP_LOGI("GpioHandler", "Read pin = %d", pin);
    return gpio_get_level(static_cast<gpio_num_t>(pin));
}