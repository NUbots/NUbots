#!/usr/bin/env python3

import enum
import math
import pdb

import tensorflow as tf

from ..matrix import compose, decompose


class CameraSystemModel(tf.keras.Model):
    def __init__(self, cameras, Hpcs, resamples, base_camera=None, dtype=tf.float64, **kwargs):
        super(CameraSystemModel, self).__init__(dtype=dtype, **kwargs)

        self.cameras = cameras
        self.resamples = resamples

        # We need to use one of the cameras as a fixed point so that all other cameras can be setup relative to it
        self.base_camera = sorted(self.cameras.keys())[0] if base_camera is None else base_camera
        self.Hbp = tf.linalg.inv(Hpcs[self.base_camera])
        self.rCBbs = {self.base_camera: tf.constant([0.0, 0.0, 0.0], dtype=self.dtype)}
        self.Rbcs = {self.base_camera: tf.constant([0.0, 0.0, 0.0], dtype=self.dtype)}

        # For every other camera, they have the full roll/pitch/yaw <x y z> degrees of freedom
        for k, Hpc in Hpcs.items():
            if k != self.base_camera:
                Hbc = tf.linalg.matmul(self.Hbp, Hpc)
                rCBb = self.add_weight("t_{}".format(k), shape=(3), initializer="zeros", dtype=self.dtype)
                rCBb.assign(Hbc[0:3, 3])

                Rbc = self.add_weight("r_{}".format(k), shape=(3), initializer="zeros", dtype=self.dtype)
                Rbc.assign(decompose(Hbc[0:3, 0:3]))

                self.rCBbs[k] = rCBb
                self.Rbcs[k] = Rbc

    def call(self, inputs, training=False):

        rPCbs = {}
        for k, camera in self.cameras.items():
            # Project the plane using the camera intrinsics
            rPCc = camera(inputs[k][0])

            # Linearly interpolate and resample for extrinsics according to the precalculated interpolations
            idx = self.resamples[k]["idx"]
            factor = self.resamples[k]["factor"]
            rPCc = tf.gather(rPCc, idx)
            rPCc = rPCc[:, 0] * (1.0 - factor) + rPCc[:, 1] * factor

            # Construct our current rotation and translation
            Rbc = compose(self.Rbcs[k])

            # Convert our vectors into the base camera coordinate system
            # Why do we transpose a here? If you work it out please tell me because I swear this is wrong but it gives the correct results
            rPCb = tf.linalg.matvec(Rbc, rPCc, transpose_a=True)
            rPCbs[k] = rPCb

        # Pair off our cameras to calculate the loss functions between them
        keys = sorted(self.cameras.keys())
        for i in range(len(keys)):
            k1 = keys[i]
            for j in range(i + 1, len(keys)):
                k2 = keys[j]

                # Gather the points seen by both cameras
                rIXb = rPCbs[k1]
                rJYb = rPCbs[k2]
                valid = tf.squeeze(
                    tf.where(
                        tf.logical_and(
                            tf.squeeze(self.resamples[k1]["factor"] != -1),
                            tf.squeeze(self.resamples[k2]["factor"] != -1),
                        )
                    ),
                )
                rIXb = tf.gather(rIXb, valid)
                rJYb = tf.gather(rJYb, valid)

                # Vector between the cameras
                rXBb = self.rCBbs[k1]
                rYBb = self.rCBbs[k2]
                rXYb = tf.broadcast_to(tf.reshape(rXBb - rYBb, [1, 1, 1, 3]), tf.shape(rIXb))

                # Calculate the dihedral angle
                dihedral_angle = self._angle_difference(
                    tf.linalg.normalize(tf.linalg.cross(rIXb, rXYb), axis=-1)[0],
                    tf.linalg.normalize(tf.linalg.cross(rJYb, rXYb), axis=-1)[0],
                )
                dihedral_loss = tf.square(dihedral_angle)

                # Get the vectors and lengths we are using to check triangularity
                uIXb, a = tf.linalg.normalize(rIXb, axis=-1)
                uJYb, b = tf.linalg.normalize(rJYb, axis=-1)
                a = tf.squeeze(a, -1)
                b = tf.squeeze(b, -1)
                c = tf.linalg.norm(rXYb, axis=-1)

                # Calculate the angles via the angle and by the distances
                # When d_angle is NaN, it means that the lengths prevented it from being a real triangle
                # As a result of this we can just assume the angle is 0 since that's the closest we can get
                # We use a numerically stable law of cosines here since we can get some very small angles
                # https://www.nayuki.io/page/numerically-stable-law-of-cosines
                a_angle = self._angle_difference(uJYb, uIXb)
                d_angle = tf.math.acos(
                    tf.clip_by_value(
                        (tf.math.square(a) + tf.math.square(b) - tf.math.square(c)) / (2.0 * a * b), -1.0, 1.0
                    )
                )
                triangle_loss = tf.math.squared_difference(a_angle, d_angle)

                # Strip out the worst losses to get rid of bad outliers
                dihedral_loss = -tf.math.top_k(
                    -tf.reshape(dihedral_loss, [-1]), k=tf.size(dihedral_loss) * 8 // 10, sorted=False
                )[0]
                triangle_loss = -tf.math.top_k(
                    -tf.reshape(triangle_loss, [-1]), k=tf.size(triangle_loss) * 8 // 10, sorted=False
                )[0]

                self.add_loss(tf.reduce_mean(dihedral_loss))
                self.add_loss(tf.reduce_mean(triangle_loss))
                self.add_metric(name="{}/{}/dihedral".format(k1, k2), value=dihedral_loss, aggregation="mean")
                self.add_metric(name="{}/{}/triangle".format(k1, k2), value=triangle_loss, aggregation="mean")

                # Calculate our expected width from each of the triangles using a numerically stable cosine rule
                # https://www.nayuki.io/page/numerically-stable-law-of-cosines
                c_error = tf.math.squared_difference(
                    c, tf.math.sqrt(tf.square(a - b) + a * b * tf.square(a_angle) * (1.0 - tf.square(a_angle) / 12.0))
                )

                # Throw out the worst 20% of values
                c_error = -tf.math.top_k(-tf.reshape(c_error, [-1]), k=tf.size(c_error) * 8 // 10, sorted=False)[0]

                self.add_metric(name="{}/{}/distance".format(k1, k2), value=c_error, aggregation="mean")

    def _angle_difference(self, u, v):
        return 2.0 * tf.math.atan2(tf.linalg.norm(u - v, axis=-1), tf.linalg.norm(u + v, axis=-1))
