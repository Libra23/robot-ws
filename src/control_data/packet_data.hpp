#ifndef PACKET_DATA_H
#define PACKET_DATA_H

#include "mainte_data.hpp"

enum PacketType {
    // mainte to robot
    MAINTE_TO_ROBOT_CONTROL_DATA,
    // robot to mainte
    ROBOT_TO_MAINTE_ROBOT_INFO
};

#pragma pack(push, 1)
struct TcpHeader {
    uint32_t size;
    uint8_t type;
    TcpHeader(uint32_t size = 0, uint8_t type = 0) : 
        size(size),
        type(type) {}
};

struct PacketControlDataReq {
    TcpHeader header;
    uint8_t arm_id;
    ControlData control_data;
    PacketControlDataReq() : 
        header(sizeof(PacketControlDataReq), MAINTE_TO_ROBOT_CONTROL_DATA),
        arm_id(ArmId::NUM_ARM) {}
};

struct PacketRobotInfoReq {
    TcpHeader header;
    uint8_t num_arm;
    uint8_t num_joint;
    PacketRobotInfoReq() : 
        header(sizeof(PacketRobotInfoReq), ROBOT_TO_MAINTE_ROBOT_INFO),
        num_arm(ArmId::NUM_ARM),
        num_joint(NUM_JOINT) {}
};
#pragma pack(pop)

#endif