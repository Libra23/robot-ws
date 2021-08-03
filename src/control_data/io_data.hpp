#ifndef IO_DATA_H
#define IO_DATA_H

#include <array>

#define NUM_GPIO            1
#define NUM_SERIAL_SERVO    12

struct GpioState {
    int8_t pin;
    bool level;
    GpioState() : 
        pin(-1),
        level(false) {}
};

struct SerialServoState {
    int8_t id;
    double act_q;
    double act_qd;
    bool enable;
    SerialServoState() : 
        id(-1),
        act_q(0.0),
        act_qd(0.0),
        enable(false) {}
};

struct InputState {
    std::array<GpioState, NUM_GPIO> gpio;
    std::array<SerialServoState, NUM_SERIAL_SERVO> serial_servo;
    InputState() :
        serial_servo{SerialServoState()} {}
};

struct OutputState {
    std::array<GpioState, NUM_GPIO> gpio;
    std::array<SerialServoState, NUM_SERIAL_SERVO> serial_servo;
    OutputState() : 
        serial_servo{SerialServoState()} {}
};

#endif