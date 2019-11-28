#!/usr/bin/env python3

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
