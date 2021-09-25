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
'''
class WaveType(IntEnum):
    CONST = 0
    SIN = 1
    RECTANGLE = 2
    TRIANGLE = 3

'''
struct WaveForm {
    int type;
    double amplitude;
    double base;
    double frequency;
    double phase;
};
'''
class WaveForm(Structure):
    _pack_ = 1
    _fields_ = [
        ('type', c_int),
        ('amplitude', c_double),
        ('base', c_double),
        ('frequency', c_double),
        ('phase', c_double)
    ]
    def __init__(self, type, amplitude, base, frequency, phase):
        self.type = type
        self.amplitude = amplitude
        self.base = base
        self.frequency = frequency
        self.phase = phase

'''
struct Reference {
    std::array<WaveForm, NUM_JOINT> fk;
    std::array<WaveForm, NUM_WRENCH> ik;
    std::array<WaveForm, NUM_JOINT> act_fk;
    std::array<bool, NUM_JOINT> enable;
};
'''
NUM_JOINT = 3
NUM_WRENCH = 6
class Reference(Structure):
    _pack_ = 1
    _fields_ = [
        ('fk', WaveForm * NUM_JOINT),
        ('ik', WaveForm * NUM_WRENCH),
        ('act_fk', WaveForm * NUM_JOINT),
        ('enable', c_bool * NUM_JOINT)
    ]
    def __init__(self):
        for i in range(NUM_JOINT):
            self.fk[i] = WaveForm(0, 0.0, 0.0, 0.0, 0.0)
            self.act_fk[i] = WaveForm(0, 0.0, 0.0, 0.0, 0.0)
            self.enable[i] = True
        for i in range(NUM_WRENCH):
            self.ik[i] = WaveForm(0, 0.0, 0.0, 0.0, 0.0)

'''
enum ControlMode {
    FK,
    IK,
    ACT_FK
};
'''
class ControlMode(IntEnum):
    FK = 0
    IK = 1
    ACT_FK = 2

'''
struct ControlData {
    int control_mode;
    std::array<Reference, NUM_ARM> reference;
};
'''
NUM_ARM = 3
class ControlData(Structure):
    _pack_ = 1
    _fields_ = [
        ('control_mode', c_int), 
        ('reference', Reference * NUM_ARM)
    ]
    def __init__(self):
        self.control_mode = ControlMode.FK
        for i in range(NUM_ARM):
            self.reference[i] = Reference()

if __name__ == '__main__':
    # check wave
    wave_form = WaveForm(0, 0.0, 0.0, 0.0, 0.0)
    
    # print(wave_form.as_dictionary())

    # check reference
    reference = Reference()
    # print(reference.as_dictionary())

    # check control data
    control_data = ControlData()
    # print(control_data.as_dictionary()) 