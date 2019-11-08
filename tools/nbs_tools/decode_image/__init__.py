#!/usr/bin/env python3

from .decode_bayer import decode_bayer
from .decode_jpeg import decode_jpeg

from .fourcc import fourcc


def decode_image(data, format):

    if format in [fourcc("JPBG"), fourcc("JPRG"), fourcc("JPGR"), fourcc("JPGB")]:
        return decode_bayer(data, format)
    elif format in [fourcc("JPEG")]:
        return decode_jpeg(data, format)
    else:
        raise RuntimeError("Unknown format {}".format(format))
