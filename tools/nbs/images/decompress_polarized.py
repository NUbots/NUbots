#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2019 NUbots
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

import tensorflow as tf

from .decompress_bayer import decompress_bayer
from .fourcc import fourcc


def decompress_polarized(data, fmt):
    bayer = decompress_bayer(
        data,
        {
            fourcc("PJRG"): fourcc("JPRG"),
            fourcc("PJGR"): fourcc("JPGR"),
            fourcc("PJBG"): fourcc("JPBG"),
            fourcc("PJGB"): fourcc("JPGB"),
        }[fmt],
        factor=4,
    )[0]

    img = tf.image.convert_image_dtype(bayer["image"], tf.float32)

    p_0 = img[0::2, 0::2]
    p_90 = img[1::2, 1::2]
    p_45 = img[0::2, 1::2]
    p_135 = img[1::2, 0::2]

    s0 = tf.math.add_n([p_0, p_90, p_45, p_135])
    s1 = tf.subtract(p_0, p_90)
    s2 = tf.subtract(p_45, p_135)

    angle = tf.multiply(tf.add(tf.atan2(s2, s1), math.pi), 1.0 / (2.0 * math.pi))
    dlop = tf.sqrt(tf.add(tf.square(s1), tf.square(s2)))
    colour = tf.multiply(s0, 0.25)
    weighted = tf.multiply(tf.multiply(angle, dlop), 10.0)

    return [
        {
            "name": "_angle",
            "image": tf.image.convert_image_dtype(angle, tf.uint8, saturate=True),
            "fourcc": bayer["fourcc"],
        },
        {
            "name": "_dlop",
            "image": tf.image.convert_image_dtype(dlop, tf.uint8, saturate=True),
            "fourcc": bayer["fourcc"],
        },
        {
            "name": "_colour",
            "image": tf.image.convert_image_dtype(colour, tf.uint8, saturate=True),
            "fourcc": bayer["fourcc"],
        },
        {
            "name": "_weighted",
            "image": tf.image.convert_image_dtype(weighted, tf.uint8, saturate=True),
            "fourcc": bayer["fourcc"],
        },
    ]
