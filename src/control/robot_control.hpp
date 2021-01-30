#ifndef ROBOT_H
#define ROBOT_H

#include "robot_data.hpp"
#include "arm_control.hpp"

#include "common/thread.hpp"
#include "io_interface/io_interface.hpp"

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
    void ConvertToOutput(const RobotRef& ref, OutputState& output_state);
    void GetDefaultRef(RobotRef& ref);
    static Vector3d GetDefaultJoint(const ArmId& id);
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