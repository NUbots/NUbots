#!/usr/bin/env python3

import math

import tensorflow as tf
import numpy as np

from ..loss import euclidean_error


def collinearity(truth, pred):
    return 90 - tf.reduce_mean(tf.acos(euclidean_error(pred)[0])) * (180 / math.pi)


def parallelity(truth, pred):
    return 90 - tf.reduce_mean(tf.acos(euclidean_error(pred)[1])) * (180 / math.pi)


def orthogonality(truth, pred):
    return 90 - tf.reduce_mean(tf.acos(euclidean_error(pred)[2])) * (180 / math.pi)


def alignment(truth, pred):
    return tf.reduce_mean(tf.acos(pred[:, :, :, 2]), axis=[1, 2]) * (180 / math.pi)


def relative_distance(truth, pred):
    return tf.reduce_mean(tf.abs(pred[..., 0:2] - truth[..., 0:2]) / truth[..., 0:2], axis=[1, 2, 3])


def absolute_distance(truth, pred):
    return tf.reduce_mean(tf.abs(pred[..., 0:2] - truth[..., 0:2]), axis=[1, 2, 3])
