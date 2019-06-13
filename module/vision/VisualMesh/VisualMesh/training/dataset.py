#!/usr/bin/env python3

import tensorflow as tf
import os
import math

# Load the visual mesh op
op_file = os.path.join(os.path.dirname(__file__), 'visualmesh_op.so')
if os.path.isfile(op_file):
  VisualMesh = tf.load_op_library(op_file).visual_mesh
else:
  raise Exception("Please build the tensorflow visual mesh op before running")


class VisualMeshDataset:

  def __init__(self, input_files, classes, geometry, batch_size, prefetch, variants):
    self.input_files = input_files
    self.classes = classes
    self.batch_size = batch_size
    self.geometry = tf.constant(geometry.shape, dtype=tf.string, name='GeometryType')
    self.prefetch = prefetch

    self._variants = variants

    # Convert our geometry into a set of numbers
    if geometry.shape in ['CIRCLE', 'SPHERE']:
      self.geometry_params = tf.constant([geometry.radius, geometry.intersections, geometry.max_distance],
                                         dtype=tf.float32,
                                         name='GeometryParams')

    elif geometry.shape in ['CYLINDER']:
      self.geometry_params = tf.constant([
        geometry.height, geometry.radius, geometry.intersections, geometry.max_distance
      ],
                                         dtype=tf.float32,
                                         name='GeometryParams')
    else:
      raise Exception('Unknown geometry type {}'.format(self.geometry))

  def _load_example(self, proto):
    example = tf.parse_single_example(
      proto, {
        'image': tf.FixedLenFeature([], tf.string),
        'mask': tf.FixedLenFeature([], tf.string),
        'lens/projection': tf.FixedLenFeature([], tf.string),
        'lens/focal_length': tf.FixedLenFeature([1], tf.float32),
        'lens/fov': tf.FixedLenFeature([1], tf.float32),
        'lens/centre': tf.FixedLenFeature([2], tf.float32),
        'mesh/orientation': tf.FixedLenFeature([3, 3], tf.float32),
        'mesh/height': tf.FixedLenFeature([1], tf.float32),
      }
    )

    return {
      'image': tf.image.decode_image(example['image'], channels=3),
      'mask': tf.image.decode_png(example['mask'], channels=4),
      'projection': example['lens/projection'],
      'focal_length': example['lens/focal_length'],
      'lens_centre': example['lens/centre'],
      'fov': example['lens/fov'],
      'orientation': example['mesh/orientation'],
      'height': example['mesh/height'],
      'raw': example['image'],
    }

  def _project_mesh(self, args):

    height = args['height']
    orientation = args['orientation']

    # Adjust our height and orientation
    if 'mesh' in self._variants:
      v = self._variants.mesh
      if 'height' in v:
        height = height + tf.truncated_normal(
          shape=(),
          mean=v.height.mean,
          stddev=v.height.stddev,
        )
      if 'rotation' in v:
        # Make 3 random euler angles
        rotation = tf.truncated_normal(
          shape=[3],
          mean=v.rotation.mean,
          stddev=v.rotation.stddev,
        )
        # Cos and sin for everyone!
        ca = tf.cos(rotation[0])
        sa = tf.sin(rotation[0])
        cb = tf.cos(rotation[1])
        sb = tf.sin(rotation[1])
        cc = tf.cos(rotation[2])
        sc = tf.sin(rotation[2])

        # Convert these into a rotation matrix
        rot = [cc*ca, -cc*sa*cb + sc*sb, cc*sa*sb + sc*cb,
                  sa,             ca*cb,         -ca * sb,
              -sc*ca,  sc*sa*cb + cc*sb, -sc*sa*sb + cc*cb]  # yapf: disable
        rot = tf.reshape(tf.stack(rot), [3, 3])

        # Apply the rotation
        orientation = tf.matmul(rot, orientation)

    # Run the visual mesh to get our values
    pixels, neighbours = VisualMesh(
      tf.shape(args['image']),
      args['projection'],
      args['focal_length'],
      args['fov'],
      args['lens_centre'],
      orientation,
      height,
      self.geometry,
      self.geometry_params,
      name='ProjectVisualMesh',
    )

    # Bilinearly interpolate our image based on our floating point pixel coordinate
    y_0 = tf.floor(pixels[:, 0])
    x_0 = tf.floor(pixels[:, 1])
    y_1 = y_0 + 1
    x_1 = x_0 + 1

    # Weights for the x and y axis
    y_w = pixels[:, 0] - y_0
    x_w = pixels[:, 1] - x_0

    # Pixel coordinates to values to weighted values to X
    p_idx = [
      tf.cast(tf.stack([a, b], axis=-1), tf.int32) for a, b in [
        (y_0, x_0),
        (y_0, x_1),
        (y_1, x_0),
        (y_1, x_1),
      ]
    ]
    p_val = [tf.image.convert_image_dtype(tf.gather_nd(args['image'], idx), tf.float32) for idx in p_idx]
    p_weighted = [
      tf.multiply(val, tf.expand_dims(w, axis=-1)) for val, w in zip(
        p_val, [
          tf.multiply(1 - y_w, 1 - x_w),
          tf.multiply(1 - y_w, x_w),
          tf.multiply(y_w, 1 - x_w),
          tf.multiply(y_w, x_w),
        ]
      )
    ]
    X = tf.add_n(p_weighted)

    # For the segmentation just use the nearest neighbour
    Y = tf.gather_nd(args['mask'], tf.cast(tf.round(pixels), tf.int32))

    return X, Y, neighbours, pixels

  def _expand_classes(self, Y):

    # Expand the classes from colours into individual columns
    W = tf.image.convert_image_dtype(Y[:, 3], tf.float32)  # Alpha channel
    cs = []
    for name, value in self.classes:
      cs.append(
        tf.where(
          tf.reduce_all(tf.equal(Y[:, :3], [value]), axis=-1),
          tf.fill([tf.shape(Y)[0]], 1.0),
          tf.fill([tf.shape(Y)[0]], 0.0),
        )
      )
    Y = tf.stack(cs, axis=-1)

    return Y, W

  def _apply_variants(self, X):
    # Make the shape of X back into an imageish shape for the functions
    X = tf.expand_dims(X, axis=0)

    # Apply the variants that were listed
    var = self._variants.image
    if 'brightness' in var and var.brightness.stddev > 0:
      X = tf.image.adjust_brightness(
        X, tf.truncated_normal(
          shape=(),
          mean=var.brightness.mean,
          stddev=var.brightness.stddev,
        )
      )
    if 'contrast' in var and var.contrast.stddev > 0:
      X = tf.image.adjust_contrast(
        X, tf.truncated_normal(
          shape=(),
          mean=var.contrast.mean,
          stddev=var.contrast.stddev,
        )
      )
    if 'hue' in var and var.hue.stddev > 0:
      X = tf.image.adjust_hue(X, tf.truncated_normal(
        shape=(),
        mean=var.hue.mean,
        stddev=var.hue.stddev,
      ))
    if 'saturation' in var and var.saturation.stddev > 0:
      X = tf.image.adjust_saturation(
        X, tf.truncated_normal(
          shape=(),
          mean=var.saturation.mean,
          stddev=var.saturation.stddev,
        )
      )
    if 'gamma' in var and var.gamma.stddev > 0:
      X = tf.image.adjust_gamma(
        X, tf.truncated_normal(
          shape=(),
          mean=var.gamma.mean,
          stddev=var.gamma.stddev,
        )
      )

    # Remove the extra dimension we added
    return tf.squeeze(X, axis=0)

  def _reduce_batch(self, protos):

    Xs = []
    Ys = []
    Ws = []
    Gs = []
    pxs = []
    ns = []
    raws = []
    n_elems = tf.zeros(shape=(), dtype=tf.int32)

    for i in range(self.batch_size):

      # Load the example from the proto
      example = self._load_example(protos[i])

      # Project the visual mesh for this example
      X, Y, G, px = self._project_mesh(example)

      # Apply any visual augmentations we may want
      if 'image' in self._variants:
        X = self._apply_variants(X)

      # Expand the classes for this value
      Y, W = self._expand_classes(Y)

      # Number of elements in this component
      n = tf.shape(Y)[0]

      # Cut off the null point and replace with -1s
      G = G[:-1] + n_elems
      G = tf.where(G == n + n_elems, tf.broadcast_to(-1, tf.shape(G)), G)

      # Move along our number of elements
      n_elems = n_elems + n

      # Add all the elements to the lists
      Xs.append(X)
      Ys.append(Y)
      Ws.append(W)
      Gs.append(G)
      pxs.append(px)
      raws.append(example['raw'])
      ns.append(n)

    # Add on the null point for X and G
    Xs.append(tf.constant([[-1.0, -1.0, -1.0]], dtype=tf.float32))
    Gs.append(tf.fill([1, 7], n_elems))

    X = tf.concat(Xs, axis=0)
    Y = tf.concat(Ys, axis=0)
    W = tf.concat(Ws, axis=0)
    G = tf.concat(Gs, axis=0)
    n = tf.stack(ns, axis=-1)
    px = tf.concat(pxs, axis=0)
    raw = tf.stack(raws, axis=0)

    # Fix the null point for G
    G = tf.where(G == -1, tf.broadcast_to(n_elems, tf.shape(G)), G)

    # Return the results
    return {'X': X, 'Y': Y, 'W': W, 'n': n, 'G': G, 'px': px, 'raw': raw}

  def build(self, stats=False):

    # Make the statistics aggregator
    if stats:
      aggregator = tf.data.experimental.StatsAggregator()

    # Load our dataset records and batch them while they are still compressed
    dataset = tf.data.TFRecordDataset(self.input_files)
    dataset = dataset.batch(self.batch_size, drop_remainder=True)

    # Apply our reduction function to project/squash our dataset into a batch
    dataset = dataset.map(self._reduce_batch, num_parallel_calls=self.prefetch)

    # Prefetch some elements to ensure training smoothness
    dataset = dataset.prefetch(self.prefetch)

    # Add the statistics
    if stats:
      options = tf.data.Options()
      options.experimental_stats.aggregator = aggregator
      dataset = dataset.with_options(options)

    return (dataset, aggregator.get_summary()) if stats else dataset
