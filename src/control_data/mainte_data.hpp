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
#pragma pack(push, 1)
struct Reference {
    std::array<WaveForm, NUM_JOINT> fk;
    std::array<WaveForm, NUM_WRENCH> ik;
    std::array<WaveForm, NUM_JOINT> act_fk;
    std::array<bool, NUM_JOINT> enable;
};
struct ControlData {
    int control_mode;
    std::array<Reference, NUM_ARM> reference;
};
#pragma pack(pop)

#endif