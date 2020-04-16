#!/usr/bin/env python3

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
