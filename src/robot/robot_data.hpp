#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

#include "algorithm/math_utility.hpp"
#include <array>

// Robot Type
#define QUAD_DIAGONAL (100)
#define QUAD_PUPPER (110)

//#define ROBOT_TYPE QUAD_DIAGONAL
#define ROBOT_TYPE QUAD_PUPPER

inline int GetRobotType() {
    return ROBOT_TYPE;
}

inline bool IsQuadDiagonal() {
    if (GetRobotType() == QUAD_DIAGONAL) {
        return true;
    } else {
        return false;
    }
}

inline bool IsQuadPupper() {
    if (GetRobotType() == QUAD_PUPPER) {
        return true;
    } else {
        return false;
    }
}

#define NUM_JOINT 3

enum ArmId {
    LEFT_FRONT,
    LEFT_BACK,
    RIGHT_FRONT,
    RIGHT_BACK,
    NUM_ARM
};

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
    BodyState body;
    std::array<ArmState, NUM_ARM> arm;
    RobotState() : 
        body(BodyState()),
        arm{ArmState()} {}
};

struct RobotRef {
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