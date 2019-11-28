#!/usr/bin/env python3

import tensorflow as tf


class Equisolid(tf.keras.Model):
    def __init__(self, focal_length, centre, **kwargs):
        super(Equisolid, self).__init__()

        self.focal_length = self.add_weight("focal_length", shape=(), initializer="ones", dtype=tf.float32)
        self.centre = self.add_weight("centre", shape=(2), initializer="zeros", dtype=tf.float32)

        self.focal_length.assign(focal_length)
        self.centre.assign(centre)

    def call(self, screen, training=False):

        offset = tf.add(screen, self.centre)
        r = tf.linalg.norm(offset, axis=-1)
        theta = 2.0 * tf.asin(tf.divide(r, tf.multiply(2.0, self.focal_length)))

        return tf.stack(
            [
                tf.math.cos(theta),
                tf.divide(tf.multiply(tf.math.sin(theta), offset[..., 0]), r),
                tf.divide(tf.multiply(tf.math.sin(theta), offset[..., 1]), r),
            ],
            axis=-1,
        )
