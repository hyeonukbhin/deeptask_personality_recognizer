#!/usr/bin/python3.5
# -*- coding: utf-8 -*-

import numpy as np

a = np.array([0,0,0])
b = np.array([1,2,3])
c = np.array([])

c = np.append(c, a)
c = np.append(c, b)

print(c)