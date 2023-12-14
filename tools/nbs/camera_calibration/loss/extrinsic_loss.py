#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2020 NUbots
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
import tensorflow as tf


def extrinsic_loss(truth, points):
    t_0 = points[..., 0]
    t_1 = points[..., 1]
    dihedral_angle = points[..., 2]
    d_0 = truth[..., 0]
    d_1 = truth[..., 1]

    e_0 = tf.reduce_mean(tf.abs(t_0 - d_0) / d_0, axis=[1, 2])
    e_1 = tf.reduce_mean(tf.abs(t_1 - d_1) / d_1, axis=[1, 2])
    a_0 = tf.reduce_mean(1.0 - dihedral_angle, axis=[1, 2])

    return e_0 + e_1 + a_0 * 20
