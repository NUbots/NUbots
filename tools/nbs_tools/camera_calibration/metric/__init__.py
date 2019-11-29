#!/usr/bin/env python3

import math

import tensorflow as tf
import numpy as np

from ..loss import euclidean_error
from ..loss.extrinsic_loss import alignment_error, grid_error


def collinearity(truth, pred):
    return 90 - tf.reduce_mean(tf.acos(euclidean_error(pred)[0])) * (180 / math.pi)


def parallelity(truth, pred):
    return 90 - tf.reduce_mean(tf.acos(euclidean_error(pred)[1])) * (180 / math.pi)


def orthogonality(truth, pred):
    return 90 - tf.reduce_mean(tf.acos(euclidean_error(pred)[2])) * (180 / math.pi)


def alignment(truth, pred):
    return tf.reduce_mean(tf.acos(tf.clip_by_value(pred[:, :, :, :, 3], -1.0, 1.0))) * (180 / math.pi)


def GridError(rows, cols, grid_size):
    # Create the grid to test against
    grid = np.empty([cols, rows, 2], dtype=np.float32)

    # Set the coordinate values in our grid
    grid[:, :, 0] = (np.arange(cols) * grid_size)[:, np.newaxis]
    grid[0::2, :, 1] = np.arange(rows) * (grid_size * 2)
    grid[1::2, :, 1] = np.arange(rows) * (grid_size * 2) + grid_size
    grid = tf.convert_to_tensor(grid)

    def grid_e(truth, pred):
        position = pred[:, :, :, :, :3]
        return tf.reduce_mean(tf.sqrt(grid_error(position, grid)))

    return grid_e
