#!/usr/bin/env python3

import tensorflow as tf

from .lens_model import LensModel


class EquisolidModel(LensModel):
    def __init__(self, **kwargs):
        super(EquisolidModel, self).__init__(**kwargs)

    def theta(self, r):
        return 2.0 * tf.asin(r / (2.0 * self.focal_length))

    def r(self, theta):
        return 2.0 * self.focal_length * tf.sin(theta * 0.5)
