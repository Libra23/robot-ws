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

# ff controller
m_ff = config['ff']['M']
c_ff = config['ff']['C']
k_ff = config['ff']['K']
if config['ff']['fc'] == 0.0:
    T = 0
else:
    T = 1 / (2 * np.pi * config['ff']['fc'])
F = tf([1], [T, 1]) # pseudo differentiate(low pass)

# fb controller
kp = config['fb']['Kp']
ki = kp * config['fb']['Ti']
kd = kp * config['fb']['Td']
if config['fb']['fc'] == 0.0:
    T = 0
else:
    T = 1 / (2 * np.pi * config['fb']['fc'])
F = tf([1], [T, 1]) # pseudo differentiate(low pass)
C_FB = tf([kd, 0], [1]) * F + tf([kp], [1]) + tf([ki], [1, 0]) # PID controller
print('FB Controller = ', C_FB)

# total model
G = C_FB * P / (1 + C_FB * P) # only negative feedback
print("Model = ", G)

G2 = feedback(C_FB * P, 1) # only negative feedback
print("Model = ", G2)

(y, t) = step(G, T = np.arange(0, 1, 0.001))
#plt.plot(t, y)
bode(G) 
bode(G2) 
plt.show()