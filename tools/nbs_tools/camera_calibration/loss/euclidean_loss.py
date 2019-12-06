#!/usr/bin/env python3

import tensorflow as tf


def euclidean_error(points):

    # Vertical lines
    v = tf.dtypes.cast(points, dtype=tf.float64)  # m_v

    # Horizontal lines
    h_1 = tf.transpose(v[:, ::2, :, :], perm=[0, 2, 1, 3])  # m_h1
    h_2 = tf.transpose(v[:, 1::2, :, :], perm=[0, 2, 1, 3])  # m_h2

    # Get the planes for each of the lines
    vp = tf.linalg.normalize(tf.linalg.cross(v[:, :, 0, :], v[:, :, -1, :]), axis=-1)[0]  # n_kv
    hp_1 = tf.linalg.normalize(tf.linalg.cross(h_1[:, :, 0, :], h_1[:, :, -1, :]), axis=-1)[0]  # n_kh1
    hp_2 = tf.linalg.normalize(tf.linalg.cross(h_2[:, :, 0, :], h_2[:, :, -1, :]), axis=-1)[0]  # n_kh2

    # Collinearity loss
    v_co = tf.reduce_mean(tf.abs(tf.einsum("abcd,abd->abc", v[:, :, 1:-1, :], vp)), axis=[1, 2])
    h1_co = tf.reduce_mean(tf.abs(tf.einsum("abcd,abd->abc", h_1[:, :, 1:-1, :], hp_1)), axis=[1, 2])
    h2_co = tf.reduce_mean(tf.abs(tf.einsum("abcd,abd->abc", h_2[:, :, 1:-1, :], hp_2)), axis=[1, 2])
    collinearity_loss = tf.multiply(tf.add_n([v_co, h1_co, h2_co]), 1.0 / 3.0)

    # Get the planes formed by each of the plane normals (parallel plane normals)
    vpp = tf.linalg.normalize(tf.linalg.cross(vp[:, 0, :], vp[:, -1, :]), axis=-1)[0]
    hpp = tf.linalg.normalize(tf.linalg.cross(hp_1[:, 0, :], hp_2[:, -1, :]), axis=-1)[0]

    # Parallelity loss
    parallelity_loss = tf.reduce_mean(
        tf.abs(
            tf.concat(
                [
                    tf.einsum("abc,ac->ab", vp[:, 1:-1, :], vpp),
                    tf.einsum("abc,ac->ab", hp_1[:, 1:, :], hpp),
                    tf.einsum("abc,ac->ab", hp_2[:, :-1, :], hpp),
                ],
                axis=-1,
            )
        ),
        axis=1,
    )

    # Orthogonality loss
    orthogonality_loss = tf.abs(tf.einsum("ab,ab->a", vpp, hpp))

    return collinearity_loss, parallelity_loss, orthogonality_loss


def euclidean_loss(_, points):
    return tf.add_n(euclidean_error(points))
