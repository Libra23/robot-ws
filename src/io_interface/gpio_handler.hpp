#ifndef GPIO_HANDLER_H
#define GPIO_HANDLER_H

#include "driver/gpio.h"

#if !defined(LOW) || !defined(HIGH)
enum OutputLevel {
    LOW,
    HIGH
};
#endif

#if !defined(INPUT) || !defined(OUTPUT) || !defined(INPUT_OUTPUT)
enum Direction {
    INPUT,
    OUTPUT,
    INPUT_OUTPUT
};
#endif

struct GpioConfig {
    uint8_t pin;
    Direction dir;
    bool open_drain;
    bool pull_up;
    bool pull_down;
    GpioConfig() : 
        pin(0),
        dir(Direction::INPUT_OUTPUT),
        open_drain(false),
        pull_up(false),
        pull_down(false) {}
};

class GpioHandler {
    public:
    GpioHandler();
    void Config(const GpioConfig& config);
    void Write(uint8_t pin, uint8_t level);
    uint8_t Read(uint8_t pin);
};

#endif