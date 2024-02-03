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

from .decompress_bayer import decompress_bayer
from .decompress_jpeg import decompress_jpeg
from .decompress_polarized import decompress_polarized
from .fourcc import fourcc, fourcc_to_string


def decode_image(data, fmt):

    # Decompress and depermute compressed bayer formats
    if fmt in [fourcc(s) for s in ("JPBG", "JPRG", "JPGR", "JPGB")]:
        return decompress_bayer(data, fmt)
    if fmt in [fourcc(s) for s in ("PJBG", "PJRG", "PJGR", "PJGB")]:
        return decompress_polarized(data, fmt)
    # JPEGs can just be decompressed
    elif fmt in [fourcc("JPEG")]:
        return decompress_jpeg(data, fmt)
    # Already raw formats can just be returned
    elif fmt in [
        fourcc(s)
        for s in (
            "BGGR",
            "RGGB",
            "GRBG",
            "GBRG",
            "RGBA",
            "RGB3",
            "RGB8",
            "BGRA",
            "BGR3",
            "BGR8",
            "GRAY",
            "GREY",
            "Y8  ",
        )
    ]:
        return [{"name": "", "image": data, "fourcc": fmt}]
    else:
        raise RuntimeError("Unknown format {}".format(fourcc_to_string(fmt)))
