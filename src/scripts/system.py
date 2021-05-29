#!/usr/bin/env python3
# coding: utf-8
from control import matlab
from matplotlib import pyplot as plt

num = [1]
den = [1, 19, 108, 180]
sys = matlab.tf(num, den)
matlab.rlocus(sys)
plt.show()