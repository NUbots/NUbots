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

import tensorflow as tf

from .decompress_bayer import decompress_bayer
from .decompress_jpeg import decompress_jpeg
from .decompress_polarized import decompress_polarized
from .fourcc import fourcc, fourcc_to_string

# The formats that are already stored as raw pixel data, and how many bytes each of their pixels takes up
raw_channels = {
    fourcc("BGGR"): 1,
    fourcc("RGGB"): 1,
    fourcc("GRBG"): 1,
    fourcc("GBRG"): 1,
    fourcc("RGBA"): 4,
    fourcc("RGB3"): 3,
    fourcc("RGB8"): 3,
    fourcc("BGRA"): 4,
    fourcc("BGR3"): 3,
    fourcc("BGR8"): 3,
    fourcc("GRAY"): 1,
    fourcc("GREY"): 1,
    fourcc("Y8  "): 1,
}


def decode_image(data, fmt, dimensions=None):
    """Decode an image into a list of {name, image, fourcc} dicts where image is a height x width x channels tensor.

    dimensions is the (width, height) of the image. It is only needed for the raw formats, as the compressed
    formats already carry their own dimensions.
    """

    # Decompress and depermute compressed bayer formats
    if fmt in [fourcc(s) for s in ("JPBG", "JPRG", "JPGR", "JPGB")]:
        return decompress_bayer(data, fmt)
    if fmt in [fourcc(s) for s in ("PJBG", "PJRG", "PJGR", "PJGB")]:
        return decompress_polarized(data, fmt)
    # JPEGs can just be decompressed
    elif fmt in [fourcc("JPEG")]:
        return decompress_jpeg(data, fmt)
    # Already raw formats just need their shape put back on, as the packet stores them as a flat run of bytes
    elif fmt in raw_channels:
        if dimensions is None:
            raise RuntimeError(
                "The image dimensions are needed to decode the raw format {}".format(fourcc_to_string(fmt))
            )

        width, height = dimensions
        image = tf.reshape(tf.io.decode_raw(data, tf.uint8), (height, width, raw_channels[fmt]))

        return [{"name": "", "image": image, "fourcc": fmt}]
    else:
        raise RuntimeError("Unknown format {}".format(fourcc_to_string(fmt)))
