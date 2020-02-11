#!/usr/bin/env python3

import tensorflow as tf

from .lens_model import LensModel


class EquisolidModel(LensModel):
    def __init__(self, **kwargs):
        super(EquisolidModel, self).__init__(**kwargs)

    def theta(self, r):
        return 2.0 * tf.asin(tf.divide(r, tf.multiply(2.0, self.focal_length)))
