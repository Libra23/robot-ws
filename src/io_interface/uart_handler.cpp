#include "uart_handler.hpp"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
/**
 * @class UartHandler
 */
UartHandler::UartHandler() {
    ESP_LOGI("UartHandler", "Constructor");
}

void UartHandler::Config(const UartConfig& config) {
    // set port and pins
    port_id_ = config.port_id;
    uart_set_pin(port_id_, config.tx_pin, config.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // install driver
    buffer_size_ = config.buffer_size;
    uart_driver_install(port_id_, buffer_size_ * 2, buffer_size_ * 2, 0, nullptr, 0);
    // set parameter
    uart_config_t uart_config;
    uart_config.baud_rate = config.baud_rate;
    uart_config.data_bits = UART_DATA_8_BITS;
    switch(config.parity) {
        case UartParity::EVEN:
            uart_config.parity = UART_PARITY_EVEN;
            break;
        case UartParity::ODD:
            uart_config.parity = UART_PARITY_ODD;
            break;
        default:
            uart_config.parity = UART_PARITY_DISABLE;
            break;
    }
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_param_config(port_id_, &uart_config);
    // other parameter
    time_out_ms_ = config.time_out_ms;
}

void UartHandler::WriteBytes(const std::vector<uint8_t>& buf) {
    uart_write_bytes(port_id_, buf.data(), buf.size());
}

void UartHandler::WriteString(const std::string& str) {
    uart_write_bytes(port_id_, str.data(), str.size());
}

bool UartHandler::ReadBytes(std::vector<uint8_t>& buf) {
    std::vector<uint8_t> buf_dummy(buffer_size_);
    int length = uart_read_bytes(port_id_, buf_dummy.data(), buffer_size_, time_out_ms_/portTICK_RATE_MS);
    if (length <= 0) {
        return false;
    } else {
        buf.resize(length);
        std::copy(buf_dummy.begin(), buf_dummy.begin() + length, buf.begin());
        return true;
    }
}

void UartHandler::FlushReceive() {
    uart_flush_input(port_id_);
}

void UartHandler::WaitTransmit() {
    uart_wait_tx_done(port_id_, time_out_ms_/portTICK_RATE_MS);
}

int UartHandler::Available() {
    size_t length = 0;
    uart_get_buffered_data_len(port_id_, &length);
    return (int)length;
}