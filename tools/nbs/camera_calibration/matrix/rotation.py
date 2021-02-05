import math

import tensorflow as tf


# Decompose a rotation matrix into three euler angles
# http://www.gregslabaugh.net/publications/euler.pdf
def decompose(R):
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
        phi = 0
        if R[2, 0] == -1:
            theta = math.pi / 2.0
            psi = math.atan2(R[0, 1], R[0, 2])
        else:
            theta = -math.pi / 2.0
            psi = math.atan2(-R[0, 1], -R[0, 2])

        return [psi, theta, phi]


# http://www.gregslabaugh.net/publications/euler.pdf
def compose(euler):
    # Extract the euler angles
    psi = euler[0]
    theta = euler[1]
    phi = euler[2]

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
            tf.stack([ones, zeros, zeros], axis=0),
            tf.stack([zeros, c_psi, -s_psi], axis=0),
            tf.stack([zeros, s_psi, c_psi], axis=0),
        ],
        axis=0,
    )
    R_y = tf.stack(
        [
            tf.stack([c_theta, zeros, s_theta], axis=0),
            tf.stack([zeros, ones, zeros], axis=0),
            tf.stack([-s_theta, zeros, c_theta], axis=0),
        ],
        axis=0,
    )
    R_z = tf.stack(
        [
            tf.stack([c_phi, -s_phi, zeros], axis=0),
            tf.stack([s_phi, c_phi, zeros], axis=0),
            tf.stack([zeros, zeros, ones], axis=0),
        ],
        axis=0,
    )

    return tf.linalg.matmul(tf.linalg.matmul(R_z, R_y), R_x)
