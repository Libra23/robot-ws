#ifndef IO_DATA_H
#define IO_DATA_H

#include <array>
#include "common/memory.hpp"

#define NUM_SERIAL_SERVO 12

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
    std::array<SerialServoState, NUM_SERIAL_SERVO> serial_servo;
    InputState() :
        serial_servo{SerialServoState()} {}
};

struct OutputState {
    std::array<SerialServoState, NUM_SERIAL_SERVO> serial_servo;
    OutputState() : 
        serial_servo{SerialServoState()} {}
};

#endif