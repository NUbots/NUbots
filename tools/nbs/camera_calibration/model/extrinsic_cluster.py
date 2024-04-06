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


class ExtrinsicCluster(tf.keras.Model):

    # Decompose a rotation matrix into three euler angles
    # http://www.gregslabaugh.net/publications/euler.pdf
    def decompose(self, R):
        # Normal case
        if abs(R[2, 0]) != 1:
            theta_0 = -math.asin(R[2, 0])
            theta_1 = math.pi - theta_0

            psi_0 = math.atan2(R[2, 1] / math.cos(theta_0), R[2, 2] / math.cos(theta_0))
            psi_1 = math.atan2(R[2, 1] / math.cos(theta_1), R[2, 2] / math.cos(theta_1))

            phi_0 = math.atan2(R[1, 0] / math.cos(theta_0), R[0, 0] / math.cos(theta_0))
            phi_1 = math.atan2(R[1, 0] / math.cos(theta_1), R[0, 0] / math.cos(theta_1))

            return (
                [psi_0, theta_0, phi_0]
                if tf.linalg.norm([psi_0, theta_0, phi_0]) < tf.linalg.norm([psi_1, theta_1, phi_1])
                else [psi_1, theta_1, phi_1]
            )
        # Locked gimbal case
        else:
            if R[2, 0] == -1:
                phi = 0
                theta = math.pi / 2.0
                psi = math.atan2(R[0, 1], R[0, 2]) + phi
            else:
                phi = 0
                theta = -math.pi / 2.0
                psi = math.atan2(-R[0, 1], -R[0, 2]) - phi

            return [psi, theta, phi]

    # http://www.gregslabaugh.net/publications/euler.pdf
    def compose(self, euler):
        # Extract the euler angles
        psi = euler[:, 0]
        theta = euler[:, 1]
        phi = euler[:, 2]

        # Get the trig values
        s_psi = tf.sin(psi)
        c_psi = tf.cos(psi)
        s_theta = tf.sin(theta)
        c_theta = tf.cos(theta)
        s_phi = tf.sin(phi)
        c_phi = tf.cos(phi)

        # Ones and zeros
        ones = tf.ones_like(psi)
        zeros = tf.zeros_like(psi)

        # Stack up the values into a matrix
        R_x = tf.stack(
            [
                tf.stack([ones, zeros, zeros], axis=1),
                tf.stack([zeros, c_psi, -s_psi], axis=1),
                tf.stack([zeros, s_psi, c_psi], axis=1),
            ],
            axis=1,
        )
        R_y = tf.stack(
            [
                tf.stack([c_theta, zeros, s_theta], axis=1),
                tf.stack([zeros, ones, zeros], axis=1),
                tf.stack([-s_theta, zeros, c_theta], axis=1),
            ],
            axis=1,
        )
        R_z = tf.stack(
            [
                tf.stack([c_phi, -s_phi, zeros], axis=1),
                tf.stack([s_phi, c_phi, zeros], axis=1),
                tf.stack([zeros, zeros, ones], axis=1),
            ],
            axis=1,
        )

        return tf.linalg.matmul(tf.linalg.matmul(R_z, R_y), R_x)

    def __init__(self, Hpcs, **kwargs):
        super(ExtrinsicCluster, self).__init__(**kwargs)

        # The first camera is the camera everything else is measured relative to.
        # So whatever transform is required to make this transform identity needs to be applied to every other camera
        self.Hc_0p = tf.linalg.inv(Hpcs[0])

        # For every other camera, they have the full roll/pitch/yaw <x y z> degrees of freedom
        self.translations = []
        self.rotations = []
        for i, Hpc in enumerate(Hpcs[1:]):
            Hc_0c_n = tf.linalg.matmul(self.Hc_0p, Hpc)

            t = self.add_weight("t_{}".format(i + 1), shape=(3), initializer="zeros", dtype=self.dtype)
            r = self.add_weight("r_{}".format(i + 1), shape=(3), initializer="zeros", dtype=self.dtype)

            t.assign(Hc_0c_n[0:3, 3])
            r.assign(self.decompose(Hc_0c_n[0:3, 0:3]))

            self.translations.append(t)
            self.rotations.append(r)

    def call(self, inputs, training=False):
        # Extract the arguments
        i_0, x_0, i_1, x_1 = inputs
        i_0 = tf.squeeze(i_0, axis=1)
        i_1 = tf.squeeze(i_1, axis=1)

        # Build up the transformations for each camera with the first camera getting 0,0,0
        rotations = self.compose(tf.stack([[0, 0, 0], *self.rotations], axis=0))
        translations = tf.stack([[0, 0, 0], *self.translations], axis=0)

        # Gather the appropriate matrix for each pair
        R_0 = tf.gather(rotations, i_0)
        p_0 = tf.gather(translations, i_0)
        R_1 = tf.gather(rotations, i_1)
        p_1 = tf.gather(translations, i_1)

        # Rotate all the vectors by the corresponding rotation matrices
        v_0 = tf.einsum("aij,aklj->akli", R_0, x_0)
        v_1 = tf.einsum("aij,aklj->akli", R_1, x_1)

        # Find the perpendicular direction for these two lines
        v_2 = tf.linalg.normalize(tf.linalg.cross(v_1, v_0), axis=-1)[0]

        p_0 = tf.broadcast_to(p_0[:, tf.newaxis, tf.newaxis], tf.shape(v_0))
        p_1 = tf.broadcast_to(p_1[:, tf.newaxis, tf.newaxis], tf.shape(v_1))

        # Setup a system of linear equations to solve the points of closest approach on each line
        # https://math.stackexchange.com/questions/1993953/closest-points-between-two-lines
        rhs = p_1 - p_0
        lhs = tf.stack([v_0, -v_1, v_2], axis=-1)
        sols = tf.squeeze(tf.linalg.solve(lhs, tf.expand_dims(rhs, -1)), -1)
        t_0 = sols[..., 0]
        t_1 = sols[..., 1]

        dihedral_angle = tf.einsum(
            "ijkl,ijkl->ijk",
            tf.linalg.normalize(tf.linalg.cross(p_0 - p_1, v_0), axis=-1)[0],
            tf.linalg.normalize(tf.linalg.cross(p_0 - p_1, v_1), axis=-1)[0],
        )

        # Return this angle error and the position of the point as a single tensor
        return tf.stack([t_0, t_1, dihedral_angle], axis=-1)
