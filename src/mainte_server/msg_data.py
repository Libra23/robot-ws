#!/usr/bin/env python3
# coding: utf-8

from ctypes import *
from mainte_data import *

class MsgType(IntEnum):
    MSG_UNKNOWN = 0
    # gui to server
    MSG_GUI_TO_SERVER_CONTROL_ON = auto()
    MSG_GUI_TO_SERVER_CONTROL_OFF = auto()
    # server to gui
    MSG_SERVER_TO_GUI_ROBOT_INFO = auto()
    # terminate
    MSG_ALL_TERMINATE = auto()

class MsgHeader(Structure):
    _pack_ = 1
    _fields_ = [
        ('size', c_uint32),
        ('type', c_uint8),
    ]
    def __init__(self, size = 0, type = 0):
        self.size = size
        self.type = type

class MsgCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ('header', MsgHeader),
    ]
    def __init__(self, type = 0):
        self.header = MsgHeader(sizeof(MsgCmd), type)

class MsgCmdByte(Structure):
    _pack_ = 1
    _fields_ = [
        ('header', MsgHeader),
        ('byte', c_uint8)
    ]
    def __init__(self, type = 0, byte = 0):
        self.header = MsgHeader(sizeof(MsgCmdByte), type)
        self.byte = byte

class MsgCmdControl(Structure):
    _pack_ = 1
    _fields_ = [
        ('header', MsgHeader),
        ('arm_id', c_uint8),
        ('control_data', ControlData)
    ]
    def __init__(self, type = 0, arm_id = 0, control_data = ControlData()):
        self.header = MsgHeader(sizeof(MsgCmdControl), type)
        self.arm_id = arm_id
        self.control_data = control_data

def GetMsgType(data):
    msg_header = MsgHeader()
    memmove(addressof(msg_header), data, sizeof(msg_header))
    return msg_header.type