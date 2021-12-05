#ifndef PACKET_DATA_H
#define PACKET_DATA_H

#include "mainte_data.hpp"
#include <vector>

enum PacketType {
    // mainte to robot
    PACKET_MAINTE_TO_ROBOT_CONTROL_DATA,
    // robot to mainte
    PACKET_ROBOT_TO_MAINTE_ROBOT_INFO
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
        header(sizeof(PacketControlDataReq), PACKET_MAINTE_TO_ROBOT_CONTROL_DATA) {}
};

struct PacketRobotInfoRes {
    TcpHeader header;
    uint8_t num_arm;
    uint8_t num_joint;
    PacketRobotInfoRes() : 
        header(sizeof(PacketRobotInfoRes), PACKET_ROBOT_TO_MAINTE_ROBOT_INFO),
        num_arm(ArmId::NUM_ARM),
        num_joint(NUM_JOINT) {}
};
#pragma pack(pop)

uint8_t GetPacketType(const uint8_t* buf) {
    TcpHeader tcp_header;
    memmove(&tcp_header, buf, sizeof(tcp_header));
    return tcp_header.type;
}

const std::vector<size_t> PACKET_SIZE_ARRAY {
    sizeof(PacketControlDataReq),
    sizeof(PacketRobotInfoRes)
};

#endif