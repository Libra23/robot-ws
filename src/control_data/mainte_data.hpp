#ifndef MAINTE_DATA_H
#define MAINTE_DATA_H

#include "robot_data.hpp"
#include "algorithm/math_const.hpp"
#include "algorithm/wave.hpp"

enum ControlMode {
    FK,
    IK,
    ACT_FK
};

struct Reference {
    std::array<std::array<WaveForm, NUM_JOINT>, NUM_ARM> fk;
    std::array<std::array<WaveForm, NUM_WRENCH>, NUM_ARM> ik;
    std::array<std::array<WaveForm, NUM_JOINT>, NUM_ARM> act_fk;
};

struct MainteData {
    ControlMode control_mode;
    Reference reference;
    std::array<std::array<bool, NUM_JOINT>, NUM_ARM> enable;
};

#endif