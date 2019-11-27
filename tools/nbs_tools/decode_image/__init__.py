#!/usr/bin/env python3

from .decompress_bayer import decompress_bayer
from .decompress_jpeg import decompress_jpeg

from .fourcc import fourcc


def decode_image(data, format):

    # Decompress and depermute compressed bayer formats
    if fourcc in [fourcc(s) for s in ("JPBG", "JPRG", "JPGR", "JPGB")]:
        return decompress_bayer(data, fourcc)
    # JPEGs can just be decompressed
    elif fourcc in [fourcc("JPEG")]:
        return decompress_jpeg(data, fourcc)
    # Already raw formats can just be returned
    elif fourcc in [
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
        return [{"name": "", "image": data, "fourcc": fourcc}]
    else:
        raise RuntimeError("Unknown format {}".format(fourcc))
