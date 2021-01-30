#ifndef SERIAL_SERVO_H
#define SERIAL_SERVO_H

#include "gpio_handler.hpp"
#include "uart_handler.hpp"

struct SerialServoConfig {
    uint8_t port_id;
    int8_t tx_pin;
    int8_t rx_pin;
    uint8_t en_pin;
    uint32_t baud_rate;
    uint32_t time_out_ms;
    SerialServoConfig() : 
        port_id(0),
        tx_pin(-1),
        rx_pin(-1),
        en_pin(0),
        baud_rate(115200),
        time_out_ms(1000) {}
};

class SerialServo {
    public:
    // krs command
    static constexpr uint32_t SERVO_MIN_VALUE = 3500;
    static constexpr uint32_t SERVO_MAX_VALUE = 11500;
    static constexpr double SERVO_MIN_DEG = -135.0;
    static constexpr double SERVO_MAX_DEG = 135.0;
    // commnad index
    static constexpr uint8_t COMMAND = 0;
    static constexpr uint8_t SUB_COMMAND = 1;
    static constexpr uint8_t POSITION_H = 1;
    static constexpr uint8_t POSITION_L = 2;
    // commnad header
    static constexpr uint8_t POSITION_CMD = 0b10000000;
    static constexpr uint8_t READ_CMD = 0b10100000;
    static constexpr uint8_t WRITE_CMD = 0b11000000;
    static constexpr uint8_t ID_CMD = 0b11100000;
    // sub command
    static constexpr uint8_t EEPROM_SC = 0x00;
    static constexpr uint8_t STRC_SC = 0x01;
    static constexpr uint8_t SPD_SC = 0x02;
    static constexpr uint8_t CUR_SC = 0x03;
    static constexpr uint8_t TMP_SC = 0x04;
    static constexpr uint8_t ANGLE_SC = 0x05;

    SerialServo();
    void Config(const SerialServoConfig& config);
    void SetPosition(uint8_t id, double deg);
    double GetPosition(uint8_t id);
    double FreePosition(uint8_t id);
    void Passive(int8_t id);
    private:
    UartHandler uart_handler_;
    GpioHandler gpio_handler_;
    uint8_t en_pin_;
    bool Synchronize(const std::vector<uint8_t>& tx, std::vector<uint8_t>& rx);
    int ConvertToValue(double deg);
    double ConvertToDeg(int value);
};

#endif