#ifndef MAINTE_DATA_H
#define MAINTE_DATA_H

#include "constant/robot_const.hpp"
#include "constant/math_const.hpp"
#include "algorithm/wave.hpp"
#include <array>

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
};

struct ControlData {
    uint8_t control_mode;
    Reference reference;
    std::array<uint8_t, NUM_JOINT> enable;
};

struct ArmInfo {
    std::array<double, XYZ> pos;
    std::array<double, RPY> rot;
    std::array<double, NUM_JOINT> q;
};

struct BodyInfo {
    std::array<double, XYZ> pos;
    std::array<double, RPY> rot;
};
#pragma pack(pop)

#endif