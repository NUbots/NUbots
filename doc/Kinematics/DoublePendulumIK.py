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
import numpy as np


def jacobian(theta):
    theta1 = theta[0]
    theta2 = theta[1]
    costheta1 = np.cos(theta1)
    sintheta1 = np.sin(theta1)
    coscombo = np.cos(theta1 + theta2)
    sincombo = np.sin(theta1 + theta2)

    return np.array([[costheta1 + coscombo, sintheta1 + sincombo], [coscombo, sincombo]])


def position(theta):
    theta1 = theta[0]
    theta2 = theta[1]
    costheta1 = np.cos(theta1)
    sintheta1 = np.sin(theta1)
    coscombo = np.cos(theta1 + theta2)
    sincombo = np.sin(theta1 + theta2)

    return np.array([sincombo + sintheta1, -coscombo - costheta1])


J = []
X = np.array([0, 0])
A = np.array([0, 0])

# dA2

X_goal = np.array([1, 0.4])

for i in range(100):
    X = position(A)
    dX = X_goal - X
    if np.linalg.norm(dX) < 0.0001:
        break
    J = jacobian(A)

    if np.linalg.det(J) != 0:
        dA = np.dot(np.linalg.inv(J), dX)
    else:
        dA = dX
    print("error = ", np.linalg.norm(dX), "Angles = ", A, "Iterations = ", i + 1)

    A = dA + A

print("Goal result = ", X_goal)
print("Final result = ", X)
print("Error = ", X_goal - X)
print("Final angles = ", A)
