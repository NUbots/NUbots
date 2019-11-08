#!/usr/bin/env python3

import tensorflow as tf


def decode_jpeg(data, format):
    img = tf.io.decode_jpeg(data)
    img = tf.image.rot90(img, k=2)

    return [{"name": "", "image": img, "pixel_format": "gray" if tf.shape(img)[2] == 1 else "rgb24"}]
