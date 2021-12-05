#ifndef MSG_DATA_H
#define MSG_DATA_H

#include "mainte_data.hpp"
#include "common/utility.hpp"
#include <algorithm>
#include <vector>

enum MsgType {
    // mainte to robot
    MSG_MAINTE_TO_ROBOT_CONTROL_ON,
    MSG_MAINTE_TO_ROBOT_CONTROL_OFF,
    // robot to mainte
    MSG_ROBOT_TO_MAINTE_ROBOT_INFO
};
#pragma pack(push, 1)
struct MsgHeader {
    uint32_t size;
    uint8_t type;
    MsgHeader(uint32_t size = 0, uint8_t type = 0) : 
        size(size),
        type(type) {}
};

struct MsgCmd {
    MsgHeader header;
    MsgCmd(uint8_t type = 0) : 
        header(sizeof(MsgCmd), type) {}
    MsgCmd(const void* buf) {
        ConvertFromBuffer(buf, this, sizeof(*this));
    }
};

struct MsgCmdByte {
    MsgHeader header;
    uint8_t byte;
    MsgCmdByte(uint8_t type = 0, uint8_t byte = 0) : 
        header(sizeof(MsgCmdByte), type),
        byte(byte) {}
    MsgCmdByte(const void* buf) {
        ConvertFromBuffer(buf, this, sizeof(*this));
    }
};

struct MsgCmdControlOn {
    MsgHeader header;
    uint8_t arm_id;
    ControlData control_data;
    MsgCmdControlOn(uint8_t arm_id = 0, ControlData control_data = ControlData()) : 
        header(sizeof(MsgCmdControlOn), MSG_MAINTE_TO_ROBOT_CONTROL_ON),
        arm_id(arm_id),
        control_data(control_data) {}
    MsgCmdControlOn(const void* buf) {
        ConvertFromBuffer(buf, this, sizeof(*this));
    }
};
#pragma pack(pop)

uint8_t GetMsgType(const void* buf) {
    MsgHeader header;
    memmove(&header, buf, sizeof(header));
    return header.type;
}

uint32_t GetMaxMsgSize() {
    const std::vector<size_t> msg_size_array {
        sizeof(MsgCmd),
        sizeof(MsgCmdByte),
        sizeof(MsgCmdControlOn)
    };
    return *std::max_element(msg_size_array.begin(), msg_size_array.end());
}


#endif