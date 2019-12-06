#!/usr/bin/env python3

import tensorflow as tf


class Equidistant(tf.keras.Model):
    def __init__(self, focal_length, centre, dtype=tf.float64, **kwargs):
        super(Equidistant, self).__init__(dtype=dtype)

        self.focal_length = self.add_weight("focal_length", shape=(), initializer="ones", dtype=self.dtype)
        self.centre = self.add_weight("centre", shape=(2), initializer="zeros", dtype=self.dtype)

        self.focal_length.assign(focal_length)
        self.centre.assign(centre)

    def call(self, screen, training=False):

        offset = tf.add(screen, self.centre)
        r = tf.linalg.norm(offset, axis=-1)
        theta = tf.divide(r, self.focal_length)

        return tf.stack(
            [
                tf.math.cos(theta),
                tf.multiply(tf.math.sin(theta), tf.divide(offset[..., 0], r)),
                tf.multiply(tf.math.sin(theta), tf.divide(offset[..., 1], r)),
            ],
            axis=-1,
        )
