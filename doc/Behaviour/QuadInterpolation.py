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
import matplotlib.pyplot as plt
import numpy as np


class Quad:
    def __init__(self, bl, tl, tr, br):
        self.bl = bl
        self.tl = tl
        self.tr = tr
        self.br = br

    def interp(self, x, y):
        return (
            (1 - x) * (1 - y) * self.bl
            + (1 - x) * (1 + y) * self.tl
            + (1 + x) * (1 + y) * self.tr
            + (1 + x) * (1 - y) * self.br
        ) / 4


bl = np.array([-3, -3])
br = np.array([3, -3])
tl = np.array([-3, 3])
tr = np.array([3, 3])

quad = Quad(bl, tl, tr, br)

# plt.figure()

x = np.linspace(-1, 1, 20)
y = x

u, v = np.array([]), np.array([])
for x_i in x:
    for y_i in y:
        u_i, v_i = quad.interp(x_i, y_i)
        u = np.append(u, [u_i])
        v = np.append(v, [v_i])

plt.plot(u, v, ".b")
axis_range = 3
plt.ylim([-axis_range, axis_range])
plt.xlim([-axis_range, axis_range])
print(u, v)
plt.show()
