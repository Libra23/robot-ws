#!/usr/bin/env python3
# coding: utf-8

from ctypes import *
from enum import *
from wave import *

NUM_ARM = 4
NUM_JOINT = 3
NUM_WRENCH = 6
XYZ = 3
RPY = 3

'''
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
'''
class ControlMode(IntEnum):
    FK = 0
    IK = 1
    ACT_FK = 2
class Reference(Structure):
    _pack_ = 1
    _fields_ = [
        ('fk', WaveForm * NUM_JOINT),
        ('ik', WaveForm * NUM_WRENCH),
        ('act_fk', WaveForm * NUM_JOINT)
    ]
class ControlData(Structure):
    _pack_ = 1
    _fields_ = [
        ('control_mode', c_uint8), 
        ('reference', Reference)
        ('enable', c_uint8 * NUM_JOINT)
    ]
class ArmInfo(Structure):
    _pack_ = 1
    _fields_ = [
        ('pos', c_double * XYZ), 
        ('rot', c_double * RPY)
        ('q', c_double * NUM_JOINT)
    ]
class BodyInfo(Structure):
    _pack_ = 1
    _fields_ = [
        ('pos', c_double * XYZ), 
        ('rot', c_double * RPY)
    ]