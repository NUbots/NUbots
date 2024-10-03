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

import tensorflow as tf


class LensModel(tf.keras.Model):
    def __init__(
        self,
        focal_length,
        centre,
        k,
        dtype=tf.float64,
        distortion_reg=1e-4,
        inverse_regularisation=10,
        inverse_parameters=9,
    ):
        super(LensModel, self).__init__(dtype=dtype)

        self.focal_length = self.add_weight("focal_length", shape=(), initializer="ones", dtype=self.dtype)
        self.centre = self.add_weight("centre", shape=(2), initializer="zeros", dtype=self.dtype)
        self.k = self.add_weight("k", shape=(len(k)), initializer="zeros", dtype=self.dtype)

        self.focal_length.assign(focal_length)
        self.centre.assign(centre)
        self.k.assign(k)

        # Often the optimisation will attempt to introduce extreme distortion on the edges that doesn't help
        self.add_loss(lambda: distortion_reg * tf.reduce_sum(tf.square(self.k)))

        # When optimising try to tweak the parameters such that our inverse function has low error
        self.add_loss(lambda: inverse_regularisation * self.inverse_quality(inverse_parameters))

    def distortion(self, r, k):

        # The polynomial
        poly = 1.0 + tf.reduce_sum(
            tf.multiply(
                tf.expand_dims(k, axis=0),
                tf.pow(
                    tf.expand_dims(r, -1), (tf.range(tf.cast(tf.size(k), tf.float64), dtype=tf.float64) + 1.0) * 2.0
                ),
            ),
            axis=-1,
        )

        # Multiply r by polynomial
        return r * poly

    def inverse_coeffs(self):
        # Extract constants
        n_k = self.k.shape[0]
        k1 = self.k[0] if n_k >= 1 else 0
        k2 = self.k[1] if n_k >= 2 else 0
        k3 = self.k[2] if n_k >= 3 else 0
        k4 = self.k[3] if n_k >= 4 else 0

        # Uses the math from the paper
        # An Exact Formula for Calculating Inverse Radial Lens Distortions
        # https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4934233/pdf/sensors-16-00807.pdf

        # fmt: off
        return [
            -k1,
            3*k1**2 - k2,
            -12*k1**3 + 8*k1*k2 - k3,
            55*k1**4 - 55*k1**2*k2 + 5*k2**2 + 10*k1*k3 - k4,
            -273*k1**5 + 364*k1**3*k2 - 78*k1*k2**2 - 78*k1**2*k3 + 12*k2*k3 + 12*k1*k4,
            1428*k1**6 - 2380*k1**4*k2 + 840*k1**2*k2**2 - 35*k2**3 + 560*k1**3*k3 - 210*k1*k2*k3 + 7*k3**2 - 105*k1**2*k4 + 14*k2*k4,
            -7752*k1**7 + 15504*k1**5*k2 - 7752*k1**3*k2**2 + 816*k1*k2**3 - 3876*k1**4*k3 + 2448*k1**2*k2*k3 - 136*k2**2*k3 - 136*k1*k3**2+ 816*k1**3*k4 - 272*k1*k2*k4 + 16*k3*k4,
            43263*k1**8 - 100947*k1**6*k2 + 65835*k1**4*k2**2 - 11970*k1**2*k2**3 + 285*k2**4 + 26334*k1**5*k3 - 23940*k1**3*k2*k3+ 3420*k1*k2**2*k3 + 1710*k1**2*k3**2 - 171*k2*k3**2 - 5985*k1**4*k4 + 3420*k1**2*k2*k4 - 171*k2**2*k4 - 342*k1*k3*k4 + 9*k4**2,
            -246675*k1**9 + 657800*k1**7*k2 - 531300*k1**5*k2**2 + 141680*k1**3*k2**3 - 8855*k1*k2**4 - 177100*k1**6*k3+ 212520*k1**4*k2*k3 - 53130*k1**2*k2**2*k3 + 1540*k2**3*k3 - 17710*k1**3*k3**2 + 4620*k1*k2*k3**2 - 70*k3**3 + 42504*k1**5*k4- 35420*k1**3*k2*k4 + 4620*k1*k2**2*k4 + 4620*k1**2*k3*k4 - 420*k2*k3*k4 - 210*k1*k4**2,
        ]
        # fmt: on

    def inverse_quality(self, n_params=9):

        # Up to sqrt 2 would work for a square image, and it's unlikely to find a camera taller than wide
        r_d = tf.cast(tf.linspace(0.0, 0.5 * math.sqrt(2), 1000), dtype=self.dtype)
        r_u = self.distortion(r_d, self.k)
        r_p = self.distortion(r_u, self.inverse_coeffs()[:n_params])
        return tf.reduce_mean(tf.math.squared_difference(r_d, r_p))

    def call(self, screen, training=False):
        # We use pi/2 a lot
        pi_2 = tf.cast(math.pi * 0.5, self.dtype)

        # Unproject the pixel coordinates back into unit vector rays
        offset = tf.add(screen, self.centre)
        r_d = tf.linalg.norm(offset, axis=-1, keepdims=True)
        r = self.distortion(r_d, self.k)
        theta = self.theta(r)
        rays = tf.concat([tf.math.cos(theta), tf.math.sin(theta) * offset / r_d], axis=-1)

        # Vertical lines
        v = rays  # m_v

        # Horizontal lines
        h_1 = tf.transpose(v[:, ::2, :, :], perm=[0, 2, 1, 3])  # m_h1
        h_2 = tf.transpose(v[:, 1::2, :, :], perm=[0, 2, 1, 3])  # m_h2

        # Get the planes for each of the lines
        vp = tf.linalg.normalize(tf.linalg.cross(v[:, :, 0, :], v[:, :, -1, :]), axis=-1)[0]  # n_kv
        hp_1 = tf.linalg.normalize(tf.linalg.cross(h_1[:, :, 0, :], h_1[:, :, -1, :]), axis=-1)[0]  # n_kh1
        hp_2 = tf.linalg.normalize(tf.linalg.cross(h_2[:, :, 0, :], h_2[:, :, -1, :]), axis=-1)[0]  # n_kh2

        # Collinearity loss, make sure we take the mean angle
        v_co = tf.reduce_mean(
            tf.square(tf.subtract(pi_2, tf.acos(tf.einsum("abcd,abd->abc", v[:, :, 1:-1, :], vp)))), axis=[1, 2]
        )
        h1_co = tf.reduce_mean(
            tf.square(tf.subtract(pi_2, tf.acos(tf.einsum("abcd,abd->abc", h_1[:, :, 1:-1, :], hp_1)))), axis=[1, 2]
        )
        h2_co = tf.reduce_mean(
            tf.square(tf.subtract(pi_2, tf.acos(tf.einsum("abcd,abd->abc", h_2[:, :, 1:-1, :], hp_2)))), axis=[1, 2]
        )
        collinearity_loss = tf.math.accumulate_n([v_co, h1_co, h2_co]) / 3.0

        # Get the planes formed by each of the plane normals (parallel plane normals)
        vpp = tf.linalg.normalize(tf.linalg.cross(vp[:, 0, :], vp[:, -1, :]), axis=-1)[0]
        hpp = tf.linalg.normalize(tf.linalg.cross(hp_1[:, 0, :], hp_2[:, -1, :]), axis=-1)[0]

        # Parallelity loss
        parallelity_loss = tf.reduce_mean(
            tf.square(
                tf.subtract(
                    pi_2,
                    tf.acos(
                        tf.concat(
                            [
                                tf.einsum("abc,ac->ab", vp[:, 1:-1, :], vpp),
                                tf.einsum("abc,ac->ab", hp_1[:, 1:, :], hpp),
                                tf.einsum("abc,ac->ab", hp_2[:, :-1, :], hpp),
                            ],
                            axis=-1,
                        )
                    ),
                )
            ),
            axis=1,
        )

        # Orthogonality loss
        orthogonality_loss = tf.square(tf.subtract(pi_2, tf.acos(tf.abs(tf.einsum("ab,ab->a", vpp, hpp)))))

        # Strip out the worst of values to help with outliers
        n_elements = tf.size(collinearity_loss) * 8 // 10
        collinearity_loss = -tf.math.top_k(-collinearity_loss, k=n_elements, sorted=False)[0]
        parallelity_loss = -tf.math.top_k(-parallelity_loss, k=n_elements, sorted=False)[0]
        orthogonality_loss = -tf.math.top_k(-orthogonality_loss, k=n_elements, sorted=False)[0]

        # Add these losses to the model
        self.add_loss(tf.reduce_mean(collinearity_loss))
        self.add_loss(tf.reduce_mean(parallelity_loss))
        self.add_loss(tf.reduce_mean(orthogonality_loss))
        self.add_metric(name="collinearity", value=collinearity_loss, aggregation="mean")
        self.add_metric(name="parallelity", value=parallelity_loss, aggregation="mean")
        self.add_metric(name="orthogonality", value=orthogonality_loss, aggregation="mean")

        return rays
