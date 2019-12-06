#!/usr/bin/env python3

import tensorflow as tf
import numpy as np


def extrinsic_loss(truth, points):
    t_0 = points[..., 0]
    t_1 = points[..., 1]
    dihedral_angle = points[..., 2]
    d_0 = truth[..., 0]
    d_1 = truth[..., 1]

    e_0 = tf.reduce_mean(tf.abs(t_0 - d_0) / d_0, axis=[1, 2])
    e_1 = tf.reduce_mean(tf.abs(t_1 - d_1) / d_1, axis=[1, 2])
    a_0 = tf.reduce_mean(1.0 - dihedral_angle, axis=[1, 2])

    return e_0 + e_1 + a_0 * 10
