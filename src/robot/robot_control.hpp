#ifndef ROBOT_H
#define ROBOT_H

#include "control_data/robot_data.hpp"
#include "control_data/io_data.hpp"
#include "arm_control.hpp"

#include "common/thread.hpp"

struct RobotConfig {
    std::array<double, XYZ> cog;
    std::array<ArmConfig, NUM_ARM> arm_config;
    RobotConfig() : 
        cog{0.0},
        arm_config{ArmConfig()} {}
};

class Robot {
    public:
    Robot();
    void Thread();
    private:
    void CreateConfig(RobotConfig& config);
    void UpdateState(RobotState& state);
    void UpdateRef(RobotOut& out);
    void ConvertOutput(const RobotRef& ref, OutputState& output);
    void ConvertInput(const InputState& input, RobotState& state);
    void GetDefaultRef(RobotRef& ref);
    void ReactReceivedMsg();
    std::array<Arm, NUM_ARM> arm_;
    RobotConfig config_;
};

class RobotMain {
    public:
    RobotMain();
    void Run();
    uint32_t StackMargin();
    private:
    static void LaunchThread(void* arg);
    Robot robot_;
    Thread th_;
};

#endif