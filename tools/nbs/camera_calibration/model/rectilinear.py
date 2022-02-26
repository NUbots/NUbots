#!/usr/bin/env python3

import tensorflow as tf

from .lens_model import LensModel


class RectilinearModel(LensModel):
    def __init__(self, **kwargs):
        super(RectilinearModel, self).__init__(**kwargs)

    def theta(self, r):
        return tf.atan(r / self.focal_length)

    def r(self, theta):
        return self.focal_length * tf.tan(theta)
