#!/usr/bin/env python3

import math
import tensorflow as tf


def build_network(X, G, groups, activation_fn):

  # Work out our activation function
  activation_fn = {
    'selu': tf.nn.selu,
    'elu': tf.nn.elu,
    'relu': tf.nn.relu,
    'relu6': tf.nn.relu6,
    'leaky_relu': tf.nn.leaky_relu,
    'swish': tf.nn.swish,
    'sigmoid': tf.nn.sigmoid,
    'tanh': tf.tanh,
  }[activation_fn]

  # Build our tensor
  logits = X
  for i, c in enumerate(groups):

    # Which convolution we are on
    with tf.variable_scope('Conv_{}'.format(i)):

      # The size of the previous output is the size of our input
      prev_last_out = logits.shape[1].value

      with tf.variable_scope('GatherConvolution'):
        # Gather our pixel coordinates based on the graph and flatten them into features
        # Don't worry about the tensorflow warning this creates, this tensor is better as a dense tensor anyway
        logits = tf.reshape(tf.gather(logits, G, name='NetworkGather'), shape=[-1, prev_last_out * G.shape[1].value])

      for j, out_s in enumerate(c):

        # Our input size is the previous output size
        in_s = logits.get_shape()[1].value

        # Which layer we are on
        with tf.variable_scope('Layer_{}'.format(j), reuse=tf.AUTO_REUSE):

          # Create weights and biases
          with tf.device('/device:CPU:0'):
            W = tf.get_variable(
              'Weights',
              shape=[in_s, out_s],
              initializer=tf.random_normal_initializer(stddev=math.sqrt(1.0 / in_s)),
              dtype=tf.float32
            )

            b = tf.get_variable(
              'Biases',
              shape=[out_s],
              initializer=tf.random_normal_initializer(stddev=math.sqrt(1.0 / out_s)),
              dtype=tf.float32
            )

          # Apply our weights and biases
          logits = tf.nn.xw_plus_b(logits, W, b)

          # Apply our activation function except for the last layer
          if i + 1 < len(groups) or j + 1 < len(c):
            logits = activation_fn(logits)

  return logits
