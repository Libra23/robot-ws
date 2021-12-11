#!/usr/bin/env python3
# coding: utf-8

from ctypes import *
from enum import *

'''
enum WaveType {
    CONST,
    SIN,
    RECTANGLE,
    TRIANGLE,
};

#pragma pack(push, 1)
struct WaveForm {
    unsigned char type;
    double amplitude;
    double base;
    double frequency;
    double phase;
};
#pragma pack(pop)
'''
class WaveType(IntEnum):
    CONST = 0
    SIN = 1
    RECTANGLE = 2
    TRIANGLE = 3
class WaveForm(Structure):
    _pack_ = 1
    _fields_ = [
        ('type', c_uint8),
        ('amplitude', c_double),
        ('base', c_double),
        ('frequency', c_double),
        ('phase', c_double)
    ]