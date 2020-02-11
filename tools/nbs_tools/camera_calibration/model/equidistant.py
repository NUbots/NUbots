#!/usr/bin/env python3

import tensorflow as tf

from .lens_model import LensModel


class EquidistantModel(LensModel):
    def __init__(self, **kwargs):
        super(EquidistantModel, self).__init__(**kwargs)

    def theta(self, r):
        return tf.divide(r, self.focal_length)
