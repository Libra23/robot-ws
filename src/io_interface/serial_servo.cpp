#include "serial_servo.hpp"
#include "esp_log.h"
/**
 * @class UartHandler
 */
SerialServo::SerialServo() {}

void SerialServo::Config(const SerialServoConfig& config) {
    // uart setting
    UartConfig uart_config;
    uart_config.port_id = config.port_id;
    uart_config.tx_pin = config.tx_pin;
    uart_config.rx_pin = config.rx_pin;
    uart_config.baud_rate = config.baud_rate;
    uart_config.parity = UartParity::EVEN;
    uart_config.time_out_ms = config.time_out_ms;
    uart_handler_.Config(uart_config);
    
    // gpio setting
    GpioConfig io_config;
    en_pin_ = config.en_pin;
    io_config.pin = en_pin_;
    io_config.dir = Direction::OUTPUT;
    gpio_handler_.Config(io_config);
    
    gpio_handler_.Write(en_pin_, HIGH);
}

double SerialServo::SetPosition(uint8_t id, double deg) {
    std::vector<uint8_t> tx(3);
    std::vector<uint8_t> rx(3);
    const int value = ConvertToValue(deg);
    tx[0] = (POSITION_CMD + id);
    tx[1] = ((value >> 7)  & 0x007F);
    tx[2] = (value & 0x007F);
    Synchronize(tx, rx);
    const int ret = ((rx[1] << 7) & 0x3F80) + (rx[2] & 0x007F);
    return ConvertToDeg(ret);
}

double SerialServo::GetPosition(uint8_t id) {
    std::vector<uint8_t> tx(2);
    std::vector<uint8_t> rx(4);
    tx[0] = (READ_CMD + id);
    tx[1] = (ANGLE_SC + 0);
    Synchronize(tx, rx);
    const int ret = ((rx[2] << 7) & 0x3F80) + (rx[3] & 0x007F);
    return ConvertToDeg(ret);
}

double SerialServo::FreePosition(uint8_t id) {
    std::vector<uint8_t> tx(3);
    std::vector<uint8_t> rx(3);
    tx[0] = (POSITION_CMD + id);
    tx[1] = (0);
    tx[2] = (0);
    Synchronize(tx, rx);
    const int ret = ((rx[1] << 7) & 0x3F80) + (rx[2] & 0x007F);
    return ConvertToDeg(ret);
}

bool SerialServo::Synchronize(const std::vector<uint8_t>& tx, std::vector<uint8_t>& rx) {
    // transmit
    gpio_handler_.Write(en_pin_, HIGH); // change mode to transmit
    uart_handler_.WriteBytes(tx);
    uart_handler_.WaitTransmit();

    // receive
    uart_handler_.FlushReceive();
    gpio_handler_.Write(en_pin_, LOW);  // change mode to receive
    //uart_handler_.ReadBytes(rx);
    return false;
}

int SerialServo::ConvertToValue(double deg) {
    double ret = (SERVO_MAX_VALUE - SERVO_MIN_VALUE) / (SERVO_MAX_DEG - SERVO_MIN_DEG) * (deg - SERVO_MIN_DEG) + SERVO_MIN_VALUE;
    return static_cast<int>(ret);
}
double SerialServo::ConvertToDeg(int value) {
    return (SERVO_MAX_DEG - SERVO_MIN_DEG) / (SERVO_MAX_VALUE - SERVO_MIN_VALUE) * (value - SERVO_MIN_VALUE) + SERVO_MIN_DEG;
}