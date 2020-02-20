#!/usr/bin/env python3

import tensorflow as tf

from .fourcc import fourcc


def decompress_jpeg(data, fmt):
    img = tf.io.decode_jpeg(data)

    return [{"name": "", "image": img, "fourcc": fourcc("GRAY") if tf.shape(img)[2] == 1 else fourcc("RGB8")}]
