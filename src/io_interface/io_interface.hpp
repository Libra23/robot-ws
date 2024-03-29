#ifndef IO_INTERFACE_H
#define IO_INTERFACE_H

#include "control_data/io_data.hpp"
#include "gpio_handler.hpp"
#include "serial_servo.hpp"

#include "common/thread.hpp"

struct IoInterfaceConfig {
    SerialServoConfig serial_servo;
    IoInterfaceConfig() : 
        serial_servo(SerialServoConfig()) {}
};

class IoInterface {
    public:
    IoInterface();
    void Thread();
    private:
    void Initialize();
    void Excute();
    void CreateConfig(IoInterfaceConfig& config);
    void UpdateInput(InputState& state);
    void UpdateOutput(const OutputState& state);
    SerialServo serial_servo_;
    IoInterfaceConfig config_;
    ThreadClock clock_;
    uint64_t counter_;
    uint64_t time_;
};

class IoInterfaceMain {
    public:
    IoInterfaceMain();
    void Run();
    uint32_t StackMargin();
    private:
    static void LaunchThread(void* arg);
    IoInterface io_interface_;
    Thread th_;
};

#endif