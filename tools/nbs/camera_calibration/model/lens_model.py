#!/usr/bin/env python3

import math

import numpy as np
import tensorflow as tf

from ..grid import possible_lines


class LensModel(tf.keras.Model):
    def __init__(
        self,
        camera_name,
        focal_length,
        centre,
        k,
        grid_size,
        dtype=tf.float64,
        distortion_reg=1e-4,
        inverse_regularisation=10,
        inverse_parameters=9,
    ):
        super(LensModel, self).__init__(dtype=dtype)

        self.camera_name = camera_name

        self.focal_length = self.add_weight("focal_length", shape=(), initializer="ones", dtype=self.dtype)
        self.centre = self.add_weight("centre", shape=(2), initializer="zeros", dtype=self.dtype)
        self.k = self.add_weight("k", shape=(len(k)), initializer="zeros", dtype=self.dtype)

        self.focal_length.assign(focal_length)
        self.centre.assign(centre)
        self.k.assign(k)
        self.grid_size = grid_size

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

    def build(self, input_shape):
        # Get all the line combinations and store them for future use
        gradients = possible_lines(input_shape[1], input_shape[2])

        # Keep keys sorted to keep the ordering consistent
        keys = list(sorted(gradients.keys()))

        # For each of the lines, pad out the indices needed to have a consistent shape
        # We also make sure the first and last index are preserved as first and last
        # This will let us grab the end points of the line so we can work out the total angle subtended
        self.p_idx = []
        max_points = max([max([len(line) for line in lines]) for lines in gradients.values()])
        sum_points = input_shape[1] * input_shape[2]
        for k in keys:
            for line in gradients[k]:
                self.p_idx.append([*line[:-1], *[sum_points for i in range(max_points - len(line))], line[-1]])
        self.p_idx = tf.stack(self.p_idx, axis=0)
        self.p_count = tf.math.count_nonzero(self.p_idx < sum_points, axis=-1)
        self.p_batch = sum([sum([len(line) for line in lines]) for lines in gradients.values()])
        self.p_valid = tf.squeeze(tf.where(self.p_count > 2), -1)

        # Do the same thing for each of the parallel lines
        self.l_idx = []
        max_lines = max([len(gradients[k]) for k in keys])
        start_lines = tf.cumsum([len(gradients[k]) for k in keys], exclusive=True)
        sum_lines = sum([len(gradients[k]) for k in keys])
        for k, s in zip(keys, start_lines):
            parallel = [i for i in range(s, s + len(gradients[k]))]
            self.l_idx.append([*parallel[:-1], *[sum_lines for i in range(max_lines - len(parallel))], parallel[-1]])
        self.l_idx = tf.stack(self.l_idx, axis=0)
        self.l_count = tf.math.count_nonzero(self.l_idx < sum_lines, axis=-1)
        self.l_batch = sum_lines
        self.l_valid = tf.squeeze(tf.where(self.l_count > 2), -1)

        # Store the expected values for the orthogonality check
        self.zero_angle_idx = keys.index((1, 0))
        self.gradient_angles = [math.acos(abs(math.cos(math.atan2(k[1], k[0])))) for k in keys if k != (1, 0)]

    def call(self, screen, training=False):

        # Unproject the pixel coordinates back into unit vector rays using the camera model
        offset = tf.math.subtract(screen, self.centre)
        r_d = tf.linalg.norm(offset, axis=-1, keepdims=True)
        r = self.distortion(r_d, self.k)
        theta = self.theta(r)
        rays = tf.concat([tf.math.cos(theta), tf.math.sin(theta) * offset / r_d], axis=-1)

        # Flatten rays so that we can index it using gather
        flat = tf.reshape(rays, [tf.shape(rays)[0], -1, 3])

        # Process each of the individual lines and work out how well each one fits to a plane
        # We pad the ends with 0s so we can do this as one giant SVD operation.
        # The 0s should not influence the SVD results as they do not add/remove variance in any dimension
        p_points = tf.reshape(
            tf.gather(tf.pad(flat, [[0, 0], [0, 1], [0, 0]]), tf.reshape(self.p_idx, [-1]), axis=1),
            [-1, *self.p_idx.shape, 3],
        )
        p_normals = tf.linalg.svd(p_points)[2][..., 2]
        p_angles = tf.acos(tf.einsum("ijkl,ijl->ijk", p_points, p_normals))
        p_totals = tf.acos(tf.einsum("ijk,ijk->ij", p_points[:, :, 0], p_points[:, :, -1]))

        # Process each of the lines that have the same gradient
        # These form parallel planes so the plane formed by the normals of these will point in the direction of the line
        # We weight the results by the total subtended angle of the inputs
        l_points = tf.reshape(
            tf.gather(tf.pad(p_normals, [[0, 0], [0, 1], [0, 0]]), tf.reshape(self.l_idx, [-1]), axis=1),
            [-1, *self.l_idx.shape, 3],
        )
        h1_co = tf.reduce_mean(
            tf.square(tf.subtract(pi_2, tf.acos(tf.einsum("abcd,abd->abc", h_1[:, :, 1:-1, :], hp_1)))), axis=[1, 2]
        )
        h2_co = tf.reduce_mean(
            tf.square(tf.subtract(pi_2, tf.acos(tf.einsum("abcd,abd->abc", h_2[:, :, 1:-1, :], hp_2)))), axis=[1, 2]
        )
        collinearity_loss = tf.math.accumulate_n([v_co, h1_co, h2_co]) / 3.0

        # We can then take each of these parallel line normals and they should form the plane created by the board
        # We can check how flat this board is to ensure that the calibration isn't trying to sneak 3d distortions in
        # to hide the errors
        x_points = l_normals
        x_normals = tf.linalg.svd(x_points)[2][..., 2]
        x_angles = tf.acos(tf.einsum("ijk,ik->ij", x_points, x_normals))

        # Now that we have all the vectors from the planes, we can also check their relative angle to eachother
        # Given that we know the angles between the vectors on the physical board, we can check them against the angles
        # that we see here
        o_angles = tf.acos(
            tf.abs(
                tf.einsum(
                    "ijk,ik->ij",
                    tf.concat([x_points[:, : self.zero_angle_idx], x_points[:, self.zero_angle_idx + 1 :]], axis=1),
                    x_points[:, self.zero_angle_idx],
                )
            )
        )

        # Collinearity loss
        p_angles = tf.gather(p_angles, self.p_valid, axis=1)
        p_totals = tf.gather(p_totals, self.p_valid, axis=1)
        p_count = tf.cast(tf.gather(self.p_count, self.p_valid), p_angles.dtype)

        p_angles = tf.reduce_sum(tf.square(math.pi * 0.5 - p_angles), axis=-1)
        p_weights = tf.square(p_totals)
        p_weights = p_weights / tf.reduce_sum(p_weights) * tf.cast(tf.size(p_weights), p_weights.dtype)
        p_loss = tf.reduce_sum(p_angles * p_weights, axis=-1) / self.p_batch

        # Parallelity
        l_angles = tf.gather(l_angles, self.l_valid, axis=1)
        l_totals = tf.gather(l_totals, self.l_valid, axis=1)
        l_count = tf.cast(tf.gather(self.l_count, self.l_valid), p_angles.dtype)

        l_angles = tf.reduce_sum(tf.square(math.pi * 0.5 - l_angles), axis=-1)
        l_weights = tf.square(l_totals)
        l_weights = l_weights / tf.reduce_sum(l_weights) * tf.cast(tf.size(l_weights), l_weights.dtype)
        l_loss = tf.reduce_sum(l_angles * l_weights, axis=-1) / self.l_batch

        # Coplanarity loss
        x_angles = tf.square(math.pi * 0.5 - x_angles)
        x_weights = tf.reduce_mean(l_totals, axis=-1) * tf.reduce_mean(p_weights, axis=-1)
        x_weights = x_weights / tf.reduce_sum(x_weights) * tf.cast(tf.size(x_weights), x_weights.dtype)
        x_loss = tf.reduce_mean(x_angles * tf.expand_dims(x_weights, 1), axis=-1)

        # Orthogonality loss
        o_angles = tf.square(self.gradient_angles - o_angles)
        o_weights = x_weights
        o_loss = tf.reduce_mean(o_angles * tf.expand_dims(o_weights, 1), axis=-1)

        # Strip out the worst of values to help with outliers
        # However when stripping out values we need to make sure we are removing by relative not absolute error
        # Otherwise the larger images (which will be more sensitive to error) will be removed even if they are good
        n_elements = tf.size(p_loss) * 8 // 10
        p_loss = tf.gather(
            p_loss,
            tf.math.top_k(
                -tf.reduce_sum(tf.square(p_angles / (p_totals * p_count)), axis=-1), k=n_elements, sorted=False
            )[1],
        )
        l_loss = tf.gather(
            l_loss,
            tf.math.top_k(
                -tf.reduce_sum(tf.square(l_angles / (l_totals * l_count)), axis=-1), k=n_elements, sorted=False
            )[1],
        )
        # For these other losses, they are already going to be better handled by big images
        # so the noisiest of values will be the smallest
        x_loss = -tf.math.top_k(-x_loss, k=n_elements, sorted=False)[0]
        o_loss = -tf.math.top_k(-o_loss, k=n_elements, sorted=False)[0]

        # Add these losses to the model
        # We don't use x loss as a loss, just a metric as it's way too easy to break
        self.add_loss(tf.reduce_mean(p_loss))
        self.add_loss(tf.reduce_mean(l_loss))
        self.add_loss(tf.reduce_mean(o_loss))
        self.add_metric(name="{}/collinearity".format(self.camera_name), value=p_loss, aggregation="mean")
        self.add_metric(name="{}/parallelity".format(self.camera_name), value=l_loss, aggregation="mean")
        self.add_metric(name="{}/coplanarity".format(self.camera_name), value=x_loss, aggregation="mean")
        self.add_metric(name="{}/orthogonality".format(self.camera_name), value=o_loss, aggregation="mean")

        # Project the rays onto the orthogonal normal we obtained while doing the optimisation
        rays = rays / tf.expand_dims(tf.einsum("ijkl,il->ijk", rays, x_normals), -1)

        # Calculate the distance between the points of the rows and columns
        d1 = tf.norm(rays[:, :, :-1] - rays[:, :, 1:], axis=-1)
        d2 = tf.norm(rays[:, :-1, :] - rays[:, 1:, :], axis=-1)

        # Calculate what the scale should be for these points to make them be the correct value
        n_samples = tf.cast(d1.shape[1] * d1.shape[2] + d2.shape[1] * d2.shape[2], d1.dtype)
        scale = (
            tf.reduce_sum((self.grid_size * 2.0) / d1, axis=[1, 2], keepdims=True)
            + tf.reduce_sum((math.sqrt(2.0) * self.grid_size) / d2, axis=[1, 2], keepdims=True)
        ) / n_samples

        # Rescale the rays to be the correct distance
        rays = rays * tf.expand_dims(scale, -1)

        # Calculate the variance for the grid distances
        d1 = tf.norm(rays[:, :, :-1] - rays[:, :, 1:], axis=-1) / 2.0
        d2 = tf.norm(rays[:, :-1, :] - rays[:, 1:, :], axis=-1) / math.sqrt(2.0)

        # Calculate the variance of these measurements
        v_a = (
            tf.reduce_sum(tf.square(d1 - self.grid_size), axis=[1, 2])
            + tf.reduce_sum(tf.square(d2 - self.grid_size), axis=[1, 2])
        ) / n_samples
        self.add_metric(name="{}/grid_error".format(self.camera_name), value=v_a, aggregation="mean")

        return rays
