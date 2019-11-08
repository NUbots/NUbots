#!/usr/bin/env python3

import numpy as np
import tensorflow as tf

from .fourcc import fourcc

lookups = {}


def decode_bayer(data, format, factor=2):
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

    # Debayer, and reshape into 2D array:
    bayer = tf.reshape(tf.gather(tf.reshape(raw_img, (height * width, 1)), indices=lookup_tensor), tf.shape(raw_img))

    # Rotate the Image by 180 Degrees:
    bayer = tf.image.rot90(bayer, k=2)

    return [
        {
            "name": "",
            "image": bayer,
            "pixel_format": {
                fourcc("JPRG"): "bayer_bggr8",
                fourcc("JPGR"): "bayer_gbrg8",
                fourcc("JPBG"): "bayer_rggb8",
                fourcc("JPGB"): "bayer_grbg8",
            }[format],
        }
    ]
