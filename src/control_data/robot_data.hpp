#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

#include "constant/robot_const.hpp"
#include "algorithm/math_utility.hpp"
#include <array>

struct BodyState {
    Affine3d trans;
    BodyState() :
        trans(Affine3d::Identity()) {}
};

struct ArmState {
    VectorXd q;
    Affine3d trans;
    VectorXd act_q;
    ArmState() :
        q(VectorXd::Zero(NUM_JOINT)),
        trans(Affine3d::Identity()),
        act_q(VectorXd::Zero(NUM_JOINT)) {}
};
struct RobotState {
    uint64_t time;
    BodyState body;
    std::array<ArmState, NUM_ARM> arm;
    RobotState() : 
        body(BodyState()),
        arm{ArmState()} {}
};

struct RobotRef {
    uint64_t time;
    BodyState body;
    std::array<ArmState, NUM_ARM> arm;
    RobotRef() : 
        body(BodyState()),
        arm{ArmState()} {}
};

struct RobotOut {
    BodyState body;
    std::array<ArmState, NUM_ARM> arm;
};

#endif