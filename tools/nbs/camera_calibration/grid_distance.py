#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2020 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
import math

import numpy as np
import tensorflow as tf


def _point_distances(cam, world):

    d_1 = tf.linalg.norm(world[..., 0, :] - world[..., 1, :], axis=-1)
    d_2 = tf.linalg.norm(world[..., 1, :] - world[..., 2, :], axis=-1)

    # Dot products to get angles
    dot_12 = tf.clip_by_value(tf.einsum("ijkl,ijkl->ijk", cam[..., 0, :], cam[..., 1, :]), -1.0, 1.0)
    dot_23 = tf.clip_by_value(tf.einsum("ijkl,ijkl->ijk", cam[..., 1, :], cam[..., 2, :]), -1.0, 1.0)
    dot_13 = dot_12 * dot_23 - tf.sqrt(1.0 - tf.square(dot_12)) * tf.sqrt(1.0 - tf.square(dot_23))

    l_2 = (d_1 * d_2 * tf.sqrt(1.0 - tf.square(dot_13))) / tf.sqrt(
        (d_1 * d_2) * (1.0 - tf.square(dot_12) - tf.square(dot_23) + tf.square(dot_13))
        - tf.square(d_1) * (tf.square(dot_23) - 1.0)
        - tf.square(d_2) * (tf.square(dot_12) - 1.0)
    )
    l_1 = (l_2 * (d_1 + d_2) * tf.sqrt(1.0 - tf.square(dot_23))) / (d_2 * tf.sqrt(1.0 - tf.square(dot_13)))
    l_3 = (l_2 * (d_1 + d_2) * tf.sqrt(1.0 - tf.square(dot_12))) / (d_1 * tf.sqrt(1.0 - tf.square(dot_13)))

    return tf.stack([l_1, l_2, l_3], axis=-1)


def _distances(cam, world):

    # Horizontal points
    h_c2 = cam[:, 1:-1, :]
    h_c1 = tf.broadcast_to(cam[:, 0:1, :], h_c2.shape)
    h_c3 = tf.broadcast_to(cam[:, -1:, :], h_c2.shape)
    h_w2 = world[1:-1, :]
    h_w1 = tf.broadcast_to(world[0:1, :], h_w2.shape)
    h_w3 = tf.broadcast_to(world[-1:, :], h_w2.shape)

    h_d = _point_distances(tf.stack([h_c1, h_c2, h_c3], axis=-2), tf.stack([h_w1, h_w2, h_w3], axis=-2))
    h_d = tf.concat(
        [
            tf.reduce_mean(h_d[..., 0], axis=1, keepdims=True),
            h_d[..., 1],
            tf.reduce_mean(h_d[..., 2], axis=1, keepdims=True),
        ],
        axis=1,
    )

    # Vertical points
    v_c2 = cam[:, :, 1:-1]
    v_c1 = tf.broadcast_to(cam[:, :, 0:1], v_c2.shape)
    v_c3 = tf.broadcast_to(cam[:, :, -1:], v_c2.shape)
    v_w2 = world[:, 1:-1]
    v_w1 = tf.broadcast_to(world[:, 0:1], v_w2.shape)
    v_w3 = tf.broadcast_to(world[:, -1:], v_w2.shape)

    v_d = _point_distances(tf.stack([v_c1, v_c2, v_c3], axis=-2), tf.stack([v_w1, v_w2, v_w3], axis=-2))
    v_d = tf.concat(
        [
            tf.reduce_mean(v_d[..., 0], axis=2, keepdims=True),
            v_d[..., 1],
            tf.reduce_mean(v_d[..., 2], axis=2, keepdims=True),
        ],
        axis=2,
    )

    # Weighted average based on the relative sizes
    return h_d * (cam.shape[1] / tf.add(*cam.shape[1:3])) + v_d * (cam.shape[2] / tf.add(*cam.shape[1:3]))


def grid_distance(points, rows, cols, grid_size, name):

    # Create the grid to test against
    world_grid = np.empty([cols, rows, 2], dtype=points.dtype)

    # Set the coordinate values in our grid
    world_grid[:, :, 0] = (np.arange(cols) * grid_size)[:, np.newaxis]
    world_grid[0::2, :, 1] = np.arange(rows) * (grid_size * 2)
    world_grid[1::2, :, 1] = np.arange(rows) * (grid_size * 2) + grid_size

    world_grid = tf.convert_to_tensor(world_grid)

    # In an asymmetric circles grid we have 2 grids interleaved together
    g1 = points[:, ::2, :, :]
    g2 = points[:, 1::2, :, :]

    # The same points in the grids coordinate system
    w1 = world_grid[::2, :]
    w2 = world_grid[1::2, :]

    # Calculate the distances to the grid points
    d1 = _distances(g1, w1)
    d2 = _distances(g2, w2)

    # Join the two grids back into one
    idx = tf.scatter_nd(
        tf.concat([tf.range(0, points.shape[1], 2), tf.range(1, points.shape[1], 2)], axis=0)[:, tf.newaxis],
        tf.range(points.shape[1]),
        [points.shape[1]],
    )
    return tf.gather(tf.concat([d1, d2], axis=1), tf.broadcast_to(idx[tf.newaxis, :], points.shape[:2]), batch_dims=1)
