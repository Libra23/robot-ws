#ifndef ROBOT_H
#define ROBOT_H

#include "algorithm/MathConst.hpp"
#include "state.hpp"
#include "leg_controller.hpp"
#include "SerialServo.hpp"
#include "Sensor.hpp"

struct RobotConfig {
    std::array<double, XYZ> cog;
    std::array<LegConfig, NUM_LEG> leg_config;
};

class Robot {
    public:
    Robot();
    void Config(SerialServo& servo);
    void UpdateState(RobotState& state);
    void Control(RobotOut& out);

    // hardware config function
    void SetHardwareConfig(RobotConfig& config);
    RobotConfig& GetHardwareConfig();

    // utility function
    void GetDefaultRef(RobotRef& ref);
    static Vector3d GetDefaultJoint(const LegID& id);

    private:
    RobotConfig config_;
    std::array<Leg, NUM_LEG> leg_;
    Sensor sensor_;
};

#endif