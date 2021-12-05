#ifndef PACKET_DATA_H
#define PACKET_DATA_H

#include "mainte_data.hpp"
#include "common/utility.hpp"
#include <algorithm>
#include <vector>

enum PacketType {
    PACKET_UNKOWN,
    // mainte to robot
    PACKET_MAINTE_TO_ROBOT_CONTROL_ON,
    PACKET_MAINTE_TO_ROBOT_CONTROL_OFF,
    // robot to mainte
    PACKET_ROBOT_TO_MAINTE_ROBOT_INFO,
    // terminate
    PACKET_TERMINATE
};

#pragma pack(push, 1)
struct TcpHeader {
    uint32_t size;
    uint8_t type;
    TcpHeader(uint32_t size = 0, uint8_t type = 0) : 
        size(size),
        type(type) {}
};

struct PacketControl {
    TcpHeader header;
    uint8_t arm_id;
    ControlData control_data;
    PacketControl(uint8_t type = 0, uint8_t arm_id = 0, ControlData control_data = ControlData()) : 
        header(sizeof(PacketControl), type),
        arm_id(arm_id),
        control_data(control_data) {}
    PacketControl(const void* buf) {ConvertFromBuffer(buf, this, sizeof(*this));}
};

struct PacketRobotInfo {
    TcpHeader header;
    uint8_t num_arm;
    uint8_t num_joint;
    PacketRobotInfo() : 
        header(sizeof(PacketRobotInfo), PACKET_ROBOT_TO_MAINTE_ROBOT_INFO),
        num_arm(ArmId::NUM_ARM),
        num_joint(NUM_JOINT) {}
    PacketRobotInfo(const void* buf) {ConvertFromBuffer(buf, this, sizeof(*this));}
};
#pragma pack(pop)

inline uint8_t GetPacketType(const void* buf) {
    TcpHeader tcp_header;
    memmove(&tcp_header, buf, sizeof(tcp_header));
    return tcp_header.type;
}

inline uint32_t GetMaxPacketSize() {
    const std::vector<size_t> packet_size_array {
        sizeof(PacketControl),
        sizeof(PacketRobotInfo)
    };
    return *std::max_element(packet_size_array.begin(), packet_size_array.end());
}

#endif