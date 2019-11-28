#!/usr/bin/env python3

import math

import tensorflow as tf
import numpy as np

from ..loss import euclidean_error
from ..loss.extrinsic_loss import alignment_error, grid_error


def collinearity(truth, pred):
    return 90 - tf.acos(tf.sqrt(tf.reduce_mean(euclidean_error(pred)[0]))) * (180 / math.pi)


def parallelity(truth, pred):
    return 90 - tf.acos(tf.sqrt(tf.reduce_mean(euclidean_error(pred)[1]))) * (180 / math.pi)


def orthogonality(truth, pred):
    return 90 - tf.acos(tf.sqrt(tf.reduce_mean(euclidean_error(pred)[2]))) * (180 / math.pi)


def alignment(truth, pred):
    return tf.reduce_mean(tf.sqrt(alignment_error(pred)))


class GridError(tf.keras.metrics.Metric):
    def __init__(self, rows, cols, grid_size, name="grid", **kwargs):
        super(GridError, self).__init__(name=name, **kwargs)

        # Create the grid to test against
        grid = np.empty([cols, rows, 2], dtype=np.float32)

        # Set the coordinate values in our grid
        grid[0::2, :, 0] = np.arange(rows) * (grid_size * 2)
        grid[1::2, :, 0] = grid_size + np.arange(rows) * (grid_size * 2)
        grid[:, :, 1] = (np.arange(cols) * (grid_size * 2))[:, np.newaxis]
        self.grid = tf.convert_to_tensor(grid)

        self.err = self.add_weight("grid_error", shape=(), initializer="zeros", dtype=tf.float32)
        self.count = self.add_weight("error_count", shape=(), initializer="zeros", dtype=tf.float32)

    def __name__(self):
        return "grid"

    def update_state(self, truth, pred, sample_weight=None):
        self.err.assign_add(tf.reduce_sum(tf.sqrt(grid_error(pred, self.grid))))
        self.count.assign_add(pred.shape[0])

    def result(self):
        return self.err / self.count

    def reset_state(self):
        self.err.assign(0)
        self.count.assign(0)
