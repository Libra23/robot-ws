#!/usr/bin/env python3
# coding: utf-8

from ctypes import *
from enum import *
import json

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
    def as_dictionary(self):
        return dict((f, getattr(self, f)) for f, _ in self._fields_)

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
    def as_dictionary(self):
        d = {}
        d['fk'] = [self.fk[i].as_dictionary() for i in range(NUM_JOINT)]
        d['ik'] = [self.ik[i].as_dictionary() for i in range(NUM_WRENCH)]
        d['act_fk'] = [self.act_fk[i].as_dictionary() for i in range(NUM_JOINT)]
        d['enable'] = [self.enable[i] for i in range(NUM_JOINT)]
        return d

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
    _fields_ = [
        ('control_mode', c_int), 
        ('reference', Reference * NUM_ARM)
    ]
    def __init__(self):
        self.control_mode = ControlMode.FK
        for i in range(NUM_ARM):
            self.reference[i] = Reference()
    def as_dictionary(self):
        d = {}
        d['control_mode'] = self.control_mode
        d['reference'] =[self.reference[i].as_dictionary() for i in range(NUM_ARM)]
        return d

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

    print(json.dumps(control_data.as_dictionary()))
