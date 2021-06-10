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

# time
t = np.arange(0, 1, 0.001)

# input
u = np.ones(len(t))

# plant model (mass-spring-damper model)
m = config['plant']['M']
c = config['plant']['C']
k = config['plant']['K']
P = tf([1], [m, c, k])
print('Plant = ', P)

# disturbance model
amp = config['disturbance']['amp']
freq = config['disturbance']['freq']
d = amp * np.sin(2 * np.pi * freq * t)

# ff controller
m_ff = config['ff']['M']
c_ff = config['ff']['C']
k_ff = config['ff']['K']
if config['ff']['fc'] == 0.0:
    T = 0
else:
    T = 1 / (2 * np.pi * config['ff']['fc'])
F = tf([1], [T, 1]) # low pass
C_FF = tf([m_ff, 0, 0], [1]) * F + tf([c_ff, 0], [1]) * F + tf([k_ff], [1])
print('FF Controller = ', C_FF)

# fb controller
kp = config['fb']['Kp']
ki = kp * config['fb']['Ti']
kd = kp * config['fb']['Td']
if config['fb']['fc'] == 0.0:
    T = 0
else:
    T = 1 / (2 * np.pi * config['fb']['fc'])
F = tf([1], [T, 1]) # pseudo differentiate(low pass)
C_FB = tf([kd, 0], [1]) * F + tf([kp], [1]) + tf([ki], [1, 0]) # pid controller
print('FB Controller = ', C_FB)

# disturbance observer fb controller
P_INV = tf([config['ex_fb']['M'], 0], [1])
print("Inverse Plant = ", P_INV)
w = 2 * np.pi * config['ex_fb']['fc']
Q = tf([w * w], [1, np.sqrt(2) * w, w * w]) # 2nd butterworth
print("Q = ", Q)

# system model
G_FF = P * C_FF
# print("Model FF = ", G_FF)
(y_u, t_u, x_u) = lsim(G_FF, U=u, T=t)
(y_d, t_d, x_d) = lsim(P, U=d, T=t)
y = y_u + y_d
plt.plot(t, y, label = "FF")

G_FB = P * C_FB / (1 + P * C_FB)
# print("Model FB = ", G_FB)
(y_u, t_u, x_u) = lsim(G_FB, U=u, T=t)
(y_d, t_d, x_d) = lsim(P / (1 + P * C_FB), U=d, T=t)
y = y_u + y_d
plt.plot(t, y, label = "FB")

G_FF_FB = P * (C_FF + C_FB ) / (1 + P * C_FB)
# print("Model FB = ", G_FB)
(y_u, t_u, x_u) = lsim(G_FF_FB, U=u, T=t)
(y_d, t_d, x_d) = lsim(P / (1 + P * C_FB), U=d, T=t)
y = y_u + y_d
plt.plot(t, y, label = "FF+FB")

P_INPUT = P / (1 - Q * Q * P * P_INV)
P_DIS = P * (1 - Q) / (1 - Q * Q * P * P_INV)
G_FF_FB_EX = P_INPUT * (C_FF + C_FB ) / (1 + P_INPUT * C_FB)
(y_u, t_u, x_u) = lsim(G_FF_FB_EX, U=u, T=t)
(y_d, t_d, x_d) = lsim(P_DIS / (1 + P_INPUT * C_FB), U=d, T=t)
y = y_u + y_d
plt.plot(t, y, label = "FF+FB+EX")

# show
plt.legend()
plt.show()