#
# MIT License
#
# Copyright (c) 2016 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
import math

import matplotlib.pyplot as plt
import numpy as np

viewpoint = np.array([-1872.262, 399.786, 450.743])
tl = np.array([159.1, 1654, -1063.8])
tr = np.array([235.5, 1399.5, 1064.5])
bl = np.array([230.5, 61.96, -1045.6])
br = np.array([-178.6, 104.6, 849.7])

v_tl = tl - viewpoint
v_tr = tr - viewpoint
v_bl = bl - viewpoint
v_br = br - viewpoint

u_l = (v_tl + v_bl) / np.linalg.norm(v_tl + v_bl)
u_r = (v_tr + v_br) / np.linalg.norm(v_tr + v_br)
u_b = (v_bl + v_br) / np.linalg.norm(v_bl + v_br)
u_t = (v_tr + v_tl) / np.linalg.norm(v_tr + v_tl)

fovx = math.acos(np.dot(u_r, u_l))
fovy = math.acos(np.dot(u_t, u_b))

print(fovx, fovy)
print(180 * fovx / math.pi, 180 * fovy / math.pi)
