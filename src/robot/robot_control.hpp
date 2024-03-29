#ifndef ROBOT_H
#define ROBOT_H

#include "arm_control.hpp"
#include "motion_generator.hpp"
#include "control_data/robot_data.hpp"
#include "control_data/io_data.hpp"
#include "control_data/msg_data.hpp"

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
    void Initialize();
    void Excute();
    void CreateConfig(RobotConfig& config);
    void UpdateState(RobotState& state);
    void UpdateRef(RobotOut& out);
    void ConvertOutput(const RobotRef& ref, OutputState& output);
    void ConvertInput(const InputState& input, RobotState& state);
    void GetDefaultRef(RobotRef& ref);
    void ReactReceivedMsg();
    void ReactControlOn(int arm_id, const ControlData& control_data);
    void ReactControlOff(int arm_id);
    std::array<Arm, NUM_ARM> arm_;
    std::array<std::unique_ptr<GeneratorBase>, NUM_ARM> motion_;
    RobotConfig config_;
    ThreadClock clock_;
    uint64_t counter_;
    double control_time_;
    RobotRef ref_pre_;
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