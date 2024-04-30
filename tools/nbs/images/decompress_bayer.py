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

import numpy as np
import tensorflow as tf

from .fourcc import fourcc

lookups = {}


def decompress_bayer(data, fmt, factor=2):
    raw_img = tf.io.decode_jpeg(data, channels=1)

    # Reference Lookups by Image Shape:
    key = tuple(v for v in raw_img.shape)
    height, width, _ = key

    # Key depends on factor, so different factors would make different tables
    key = (*key, factor)

    # Make a lookup
    if key not in lookups:
        output = np.zeros(height * width, dtype=np.uint32)
        i = 0
        for y in range(0, height):
            for x in range(0, width):
                # Local x and y are the same for all within the mosaic block
                new_x = (x % factor) * (width // factor) + x // factor
                new_y = (y % factor) * (height // factor) + y // factor
                # Save and Increment
                output[i] = new_y * (width) + new_x
                i = i + 1
        lookups[key] = tf.convert_to_tensor(output, dtype=tf.int32)

    # Load the Cached Tensor:
    lookup_tensor = lookups[key]

    # Permute the mosaic, and reshape into 2D array:
    bayer = tf.reshape(tf.gather(tf.reshape(raw_img, (height * width, 1)), indices=lookup_tensor), tf.shape(raw_img))

    return [
        {
            "name": "",
            "image": bayer,
            "fourcc": {
                fourcc("JPRG"): fourcc("RGGB"),
                fourcc("JPGR"): fourcc("GRBG"),
                fourcc("JPBG"): fourcc("BGGR"),
                fourcc("JPGB"): fourcc("GBRG"),
            }[fmt],
        }
    ]
