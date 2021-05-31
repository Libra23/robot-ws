#!/usr/bin/env python3
# coding: utf-8
from control.matlab import *
from matplotlib import pyplot as plt
import yaml
import os
import numpy as np

path = os.getcwd()

print(path)

# open parameter yaml
with open(os.getcwd() + '/src/scripts/system_config.yml','r') as yml:
    config = yaml.safe_load(yml)

# plant model
m = config['plant']['M']
c = config['plant']['C']
k = config['plant']['K']
P = tf([1], [m, c, k])
print('Plant = ', P)

# fb controller
kp = config['fb']['Kp']
ki = kp * config['fb']['Ti']
kd = kp * config['fb']['Td']
T = 1 / (2 * np.pi * config['fb']['fc']) # low-pass
F = tf([1], [T, 1])
C_FB = tf([kd, 0], [1]) * F + tf([kp], [1]) + tf([ki], [1, 0])
print('FB Controller = ', C_FB)

# total model
G = feedback(C_FB * P, 1)
print("Model = ", G)

(y, t) = step(G, T = np.arange(0, 1, 0.001))
plt.plot(t, y)
plt.show()