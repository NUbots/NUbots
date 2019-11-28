#!/usr/bin/env python3

import tensorflow as tf
import numpy as np


def alignment_error(points):
    # Work out how far our lines are, they should be the same point (0 distance)
    return tf.reduce_mean(tf.square(tf.linalg.norm(points[:, 0] - points[:, 1], axis=-1)), axis=[1, 2])


def grid_error(points, grid):

    # Average the two points to find the best fit between the two points
    mean_points = tf.multiply(tf.add(points[:, 0], points[:, 1]), 0.5)

    # Subtract a point to be the new origin (0,0 should be fine)
    mean_points = tf.subtract(mean_points, mean_points[:, 0:1, 0:1])

    # Create our transformation matrix and shift the coordinates into a flat space
    x_axis = tf.linalg.normalize(mean_points[:, 0, -1], axis=-1)[0]
    y_axis = tf.linalg.normalize(mean_points[:, -1, 0], axis=-1)[0]
    z_axis = tf.linalg.normalize(tf.linalg.cross(x_axis, y_axis), axis=-1)[0]
    Hcw = tf.stack([x_axis, y_axis, z_axis], axis=-2)
    flat_points = tf.einsum("ijk,iabk->iabj", Hcw, mean_points)

    # Get the error between what the grid should actually be and what we got
    return tf.reduce_mean(tf.square(tf.norm(flat_points[..., :2] - grid, axis=-1)), axis=[1, 2])


class ExtrinsicLoss:
    def __init__(self, rows, cols, grid_size):
        # Create the grid to test against
        grid = np.empty([cols, rows, 2], dtype=np.float32)

        # Set the coordinate values in our grid
        grid[0::2, :, 0] = np.arange(rows) * (grid_size * 2)
        grid[1::2, :, 0] = grid_size + np.arange(rows) * (grid_size * 2)
        grid[:, :, 1] = (np.arange(cols) * (grid_size * 2))[:, np.newaxis]

        self.grid = tf.convert_to_tensor(grid)

    def __call__(self, _, points, sample_weight=None):
        error = tf.add(alignment_error(points), grid_error(points, self.grid))

        # Add the two errors together
        return tf.reduce_mean(tf.multiply(sample_weight, error))
