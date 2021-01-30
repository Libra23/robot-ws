#ifndef STATE_H
#define STATE_H

#include "algorithm/math_const.hpp"
#include "algorithm/kinematic.hpp"

struct BodyState {
    Affine3d trans;
};

struct LegState {
    Joint q;
    Affine3d trans;
};

struct SensorState {
    std::array<double, XYZ> accel;
    std::array<double, XYZ> gyro;
    double tof_length;
    double temperature;
    SwitchStatus switch_status;
};

struct RobotState {
    BodyState body_state;
    std::array<LegState, NUM_LEG> leg_state;
    SensorState sensor_state;
};

struct RobotRef {
    BodyState body_state;
    std::array<LegState, NUM_LEG> leg_state;
};

struct RobotOut {
    bool is_passive;
    BodyState body_state;
    std::array<LegState, NUM_LEG> leg_state;
};

#endif