#!/usr/bin/env python3
# coding: utf-8

from ctypes import *
from mainte_data import *

class PacketType(IntEnum):
    PACKET_UNKNOWN = 0
    # mainte to robot
    PACKET_MAINTE_TO_ROBOT_CONTROL_ON = auto()
    PACKET_MAINTE_TO_ROBOT_CONTROL_OFF = auto()
    # robot to mainte
    PACKET_ROBOT_TO_MAINTE_ROBOT_INFO = auto()
    # terminate
    PACKET_TERMINATE = auto()
class TcpHeader(Structure):
    _pack_ = 1
    _fields_ = [
        ('size', c_uint32),
        ('type', c_uint8)
    ]
    def __init__(self, size = 0, type = 0):
        self.size = size
        self.type = type
class PacketControl(Structure):
    _pack_ = 1
    _fields_ = [
        ('header', TcpHeader),
        ('arm_id', c_uint8),
        ('control_data', ControlData)
    ]
    def __init__(self, type = 0, arm_id = 0, control_data = ControlData()):
        self.header = TcpHeader(sizeof(PacketControl), type)
        self.arm_id = arm_id
        self.control_data = control_data
class PacketRobotInfo(Structure):
    _pack_ = 1
    _fields_ = [
        ('header', TcpHeader),
        ('num_arm', c_uint8),
        ('num_joint', c_uint8)
    ]
    def __init__(self):
        self.header = TcpHeader(sizeof(PacketRobotInfo), PacketType.PACKET_ROBOT_TO_MAINTE_ROBOT_INFO)
        self.num_arm = 0
        self.num_joint = 0

def GetPacketType(data):
    tcp_header = TcpHeader()
    memmove(addressof(tcp_header), data, sizeof(tcp_header))
    return tcp_header.type

if __name__ == '__main__':
    packet_control = PacketControl()
    print("PacketControl Size={}, Size in header = {}", sizeof(PacketControl), packet_control.header.size)
    packet_robot_info = PacketRobotInfo()
    print("PacketRobotInfo Size={}, Size in header = {}", sizeof(packet_robot_info), packet_control.header.size)