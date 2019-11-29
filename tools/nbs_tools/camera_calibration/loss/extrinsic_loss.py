#!/usr/bin/env python3

import tensorflow as tf
import numpy as np


def alignment_error(angles):
    # Average all of the angle error
    return tf.reduce_mean(tf.subtract(1.0, angles), axis=[1, 2, 3])


def grid_error(points, grid):

    # Average the two points to find the best fit between the two points
    mean_points = tf.multiply(tf.add(points[:, 0], points[:, 1]), 0.5)

    # Subtract a point to be the new origin (top left point should be fine)
    mean_points = tf.subtract(mean_points, mean_points[:, 0:1, 0:1])

    # Create our transformation matrix and shift the coordinates into a flat space
    x_axis = tf.linalg.normalize(mean_points[:, -1, 0], axis=-1)[0]
    y_axis = tf.linalg.normalize(mean_points[:, 0, -1], axis=-1)[0]
    z_axis = tf.linalg.normalize(tf.linalg.cross(x_axis, y_axis), axis=-1)[0]
    Hcw = tf.stack([x_axis, y_axis, z_axis], axis=-2)
    flat_points = tf.einsum("ijk,iabk->iabj", Hcw, mean_points)

    # Get the error between what the grid should actually be and what we got
    return tf.reduce_mean(tf.reduce_sum(tf.math.squared_difference(flat_points[..., :2], grid), axis=-1), axis=[1, 2])


def ExtrinsicLoss(rows, cols, grid_size):
    # Create the grid to test against
    grid = np.empty([cols, rows, 2], dtype=np.float32)

    # Set the coordinate values in our grid
    grid[:, :, 0] = (np.arange(cols) * grid_size)[:, np.newaxis]
    grid[0::2, :, 1] = np.arange(rows) * (grid_size * 2)
    grid[1::2, :, 1] = np.arange(rows) * (grid_size * 2) + grid_size

    grid = tf.convert_to_tensor(grid)

    def extrinsic_loss(_, points):

        # Extract the variables from the model
        angle = points[:, :, :, :, 3]
        position = points[:, :, :, :, :3]

        # Add the two errors together
        loss = tf.add(alignment_error(angle), grid_error(position, grid))
        return loss

    return extrinsic_loss
