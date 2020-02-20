#!/usr/bin/env python3

import math

import numpy as np
import tensorflow as tf


def alignment(truth, pred):
    return tf.reduce_mean(tf.acos(pred[:, :, :, 2]), axis=[1, 2]) * (180 / math.pi)


def relative_distance(truth, pred):
    return tf.reduce_mean(tf.abs(pred[..., 0:2] - truth[..., 0:2]) / truth[..., 0:2], axis=[1, 2, 3])


def absolute_distance(truth, pred):
    return tf.reduce_mean(tf.abs(pred[..., 0:2] - truth[..., 0:2]), axis=[1, 2, 3])
