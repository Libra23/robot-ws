#!/usr/bin/env python3
# coding: utf-8

from ctypes import *
from enum import *
from mainte_data import *

'''
enum PacketType {
    // mainte to robot
    MAINTE_TO_ROBOT_CONTROL_DATA,
    // robot to mainte
    ROBOT_TO_MAINTE_ARM_INFO
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
    uint8_t robot_type;
    uint8_t num_arm;
    uint8_t num_joint;
    PacketRobotInfoReq() : 
        header(sizeof(PacketRobotInfoReq), ROBOT_TO_MAINTE_ROBOT_INFO),
        num_arm(ArmId::NUM_ARM),
        num_joint(NUM_JOINT) {}
};
#pragma pack(pop)
'''
class PacketType(IntEnum):
    MAINTE_TO_ROBOT_CONTROL_DATA = 0
    ROBOT_TO_MAINTE_ARM_INFO = 1
class TcpHeader(Structure):
    _pack_ = 1
    _fields_ = [
        ('size', c_uint32),
        ('type', c_uint8)
    ]
    def __init__(self, size = 0, type = 0):
        self.size = size
        self.type = type
class PacketControlDataReq(Structure):
    _pack_ = 1
    _fields_ = [
        ('header', TcpHeader),
        ('arm_id', c_uint8),
        ('control_data', ControlData)
    ]
    def __init__(self):
        self.header = TcpHeader(sizeof(PacketControlDataReq), PacketType.MAINTE_TO_ROBOT_CONTROL_DATA)
class PacketArmInfoReq(Structure):
    _pack_ = 1
    _fields_ = [
        ('header', TcpHeader),
        ('num_arm', c_uint8),
        ('num_joint', c_uint8)
    ]
    def __init__(self):
        self.header = TcpHeader(sizeof(PacketArmInfoReq), PacketType.ROBOT_TO_MAINTE_ARM_INFO)
        self.num_arm = 0
        self.num_joint = 0