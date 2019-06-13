#!/usr/bin/env python3

import os
import copy
import numpy as np
import tensorflow as tf
from tensorflow.python.client import device_lib
from tqdm import tqdm

from . import dataset
from . import network


def _distribute_megabatch(classes, input_ops, output_ops, batches):

  # We are building up a feed_dict to pass in and an op to execute
  feed_dict = {}
  op = {}

  # Loop through the classes
  for i, c in enumerate(classes):

    # Work out which GPU we want to run this class on
    gpu_no = i % len(input_ops)

    # Set up our inputs and outputs for this class on this gpu
    feed_dict[input_ops[gpu_no][c[0]]['X']] = np.concatenate([m[c[0]]['X'] for m in batches], axis=0)
    feed_dict[input_ops[gpu_no][c[0]]['c']] = np.concatenate([m[c[0]]['c'] for m in batches], axis=0)
    op[c[0]] = {'X': output_ops[gpu_no][c[0]]['X'], 'c': output_ops[gpu_no][c[0]]['c']}

  return op, feed_dict


def _reduce_histogram_batch(X, c, num_bins):

  # Sort by confidence
  idx = tf.argsort(X)
  X = tf.gather(X, idx)
  c = tf.gather(c, idx)

  # Calculate precision and recall for all confidence thresholds
  thresholded = tf.cumsum(c, reverse=True, axis=0)
  p = tf.reduce_sum(c[:, 0], axis=0)
  precision = tf.divide(thresholded[:, 0], tf.add(thresholded[:, 0], thresholded[:, 1]))
  recall = tf.divide(thresholded[:, 0], p)

  # Calculate the distance along the PR curve for each point so we can sample evenly along the PR curve
  curve = tf.sqrt(
    tf.add(tf.squared_difference(precision[:-1], precision[1:]), tf.squared_difference(recall[:-1], recall[1:]))
  )
  curve = tf.pad(tf.cumsum(curve), [[1, 0]])  # First point is 0 length along the curve

  # Use cumsum and scatter to try to distribute points as evenly along the PR curve as we can
  idx = tf.cast(tf.expand_dims(tf.multiply(curve, tf.divide(num_bins, curve[-1])), axis=-1), dtype=tf.int32)
  idx = tf.clip_by_value(idx, 0, num_bins - 1)
  values = tf.reduce_sum(c, axis=-1)
  h_X = tf.scatter_nd(idx, tf.multiply(X, values), [num_bins])  # Scale by values in bin
  h_c = tf.scatter_nd(idx, c, [num_bins, 2])
  h_X = tf.divide(h_X, tf.reduce_sum(h_c, axis=-1))  # Renormalise by values in bin

  # Remove any points from the histogram that didn't end up getting values
  idx = tf.squeeze(tf.where(tf.logical_not(tf.is_nan(h_X))), axis=-1)
  h_X = tf.gather(h_X, idx)
  h_c = tf.gather(h_c, idx)

  # If we don't have enough values to make up the bins, just return the values themselves
  X = tf.cond(tf.size(X) < num_bins, lambda: X, lambda: h_X)
  c = tf.cond(tf.size(c) < num_bins, lambda: c, lambda: h_c)

  return X, c


def _build_device_testing_graph(data, network_structure, config, device_no):

  # Create the network graph ops for this device
  with tf.variable_scope("Network"):
    X = network.build_network(data["X"], data["G"], network_structure, config.network.activation_fn)

  # Truth labels for the network
  Y = data["Y"]

  # Use the alpha channel to strip points without labels
  W = data["W"]
  S = tf.where(tf.greater(W, 0))
  X = tf.gather_nd(X, S)
  Y = tf.gather_nd(Y, S)

  # Softmax to get actual output
  X = tf.nn.softmax(X, axis=1)

  # True and false positives (assuming 1 and 0 for true/false)
  tp = Y
  fp = 1.0 - Y

  # Calculate the precision recall for each class individually
  ops = {}
  for i, c in enumerate(config.network.classes):

    # Get the values for this specific class and the true and false positives
    X_c = X[:, i]
    tpfp = tf.stack([tp[:, i], fp[:, i]], axis=-1)

    # Store our input tensors for later so we can override them as placeholders when we reduce a series of batches together
    inputs = {'X': X_c, 'c': tpfp}

    # Reduce these points down
    X_c, tpfp = _reduce_histogram_batch(X_c, tpfp, config.testing.num_bins)

    # Store our outputs
    outputs = {'X': X_c, 'c': tpfp}

    # For the ops for this particular class
    ops[c[0]] = {'inputs': inputs, 'outputs': outputs}

  return ops


def _build_testing_graph(gpus, config):

  with tf.device("/device:CPU:0"):
    iterator = (
      dataset.VisualMeshDataset(
        input_files=config.dataset.testing,
        classes=config.network.classes,
        geometry=config.geometry,
        batch_size=max(1, config.testing.batch_size),
        prefetch=tf.data.experimental.AUTOTUNE,
        variants={},
      ).build().make_one_shot_iterator()
    )

  # Calculate the structure for the network and the tutor
  network_structure = copy.deepcopy(config.network.structure)
  network_structure[-1].append(len(config.network.classes))

  # For each GPU build a classification network, a tutor network and a gradients calculator
  device_ops = []
  for i, gpu in enumerate(gpus):
    with tf.device(gpu), tf.name_scope("Tower_{}".format(i)):
      device_ops.append(_build_device_testing_graph(iterator.get_next(), network_structure, config, i))

  return device_ops


# Train the network
def test(config, output_path):

  # Find the GPUs we have available and if we don't have any, fallback to CPU
  gpus = [x.name for x in device_lib.list_local_devices() if x.device_type == "GPU"]
  gpus = ["/device:CPU:0"] if len(gpus) == 0 else gpus

  # Build the training graph operations we need
  ops = _build_testing_graph(gpus, config)

  # Split our ops into input and output ops for each device
  input_ops = [{k: v['inputs'] for k, v in d.items()} for d in ops]
  output_ops = [{k: v['outputs'] for k, v in d.items()} for d in ops]

  # Tensorflow configuration
  tf_config = tf.ConfigProto()
  tf_config.allow_soft_placement = False
  tf_config.gpu_options.allow_growth = True

  with tf.Session(config=tf_config) as sess:
    # Initialise global variables
    sess.run(tf.global_variables_initializer())

    save_vars = {v.name: v for v in tf.trainable_variables()}
    saver = tf.train.Saver(save_vars)

    # Get our model directory and load it
    checkpoint_file = tf.train.latest_checkpoint(output_path)
    print("Loading model {}".format(checkpoint_file))
    saver.restore(sess, checkpoint_file)

    # Count how many files are in the test dataset so we can show a progress bar
    # This is slow, but the progress bar is comforting
    print("Counting records in test dataset")
    num_records = sum(1 for _ in tf.python_io.tf_record_iterator(config.dataset.testing))
    num_batches = num_records // max(1, config.testing.batch_size)
    print("Loading {} records in {} batches".format(num_records, num_batches))

    # Process the results in batches, reducing the number of points down for tp and fp
    batches = []
    with tqdm(total=num_records // max(1, config.testing.batch_size), dynamic_ncols=True, unit='batch') as progress:
      try:
        while num_batches - len(batches) > 0:
          # Accumulate our individual batches
          batch = sess.run(output_ops[0:min(num_batches - len(batches), len(output_ops))])
          batches.extend(batch)
          progress.update(len(batch))

      except tf.errors.OutOfRangeError:
        print('Loaded into {} batches'.format(len(batches)))

    # Feed the megabatches and batches together into a merger
    op, feed_dict = _distribute_megabatch(config.network.classes, input_ops, output_ops, batches)
    hist = sess.run(op, feed_dict=feed_dict)

    # Process each class into a PR curve and save
    print('Processing PR curves')
    os.makedirs(os.path.join(output_path, 'test'), exist_ok=True)
    aps = []
    for k, data in tqdm(hist.items(), dynamic_ncols=True, unit='class'):

      # Concatenate the histogram thresholds into a set
      X = data['X']
      tpfp = data['c']
      p = np.sum(tpfp[:, 0])

      # Sort by threshold
      idx = np.argsort(X)
      X = X[idx]
      tpfp = tpfp[idx]

      # This is the precision when all points are selected (used for recall = 1 endpoint)
      all_points_precision = tpfp[:, 0].sum() / (tpfp[:, 0].sum() + tpfp[:, 1].sum())

      # Cumulative sum to calculate tp and fp for each threshold
      tp = np.cumsum(tpfp[::-1, 0], dtype=np.int64)[::-1]
      fp = np.cumsum(tpfp[::-1, 1], dtype=np.int64)[::-1]

      # Calculate precision and recall
      precision = np.true_divide(tp, (tp + fp))
      recall = np.true_divide(tp, p)

      # Add on the ends of the plot to make it cleaner and make the AP more accurate
      X = np.concatenate((X, [1.0]))
      precision = np.concatenate((precision, [1.0]))
      recall = np.concatenate((recall, [0.0]))
      if recall.max() < 1.0:
        X = np.concatenate((X, [0.0]))
        precision = np.concatenate((precision, [all_points_precision]))
        recall = np.concatenate((recall, [1.0]))

      # Sort by recall
      idx = np.argsort(recall)
      X = X[idx]
      precision = precision[idx]
      recall = recall[idx]

      # Calculate the average precision using this curve
      ap = np.trapz(precision, recall)

      # Add this average precision to the summary
      aps.append((k.title(), ap))

      # Save the PR curve
      np.savetxt(
        os.path.join(output_path, 'test', '{}_pr.csv'.format(k)),
        np.stack([X, recall, precision], axis=-1),
        comments="",
        header="Confidence,Recall,Precision",
        delimiter=",",
      )

    # Write out the mAP and ap for each class
    with open(os.path.join(output_path, 'test', 'pr.txt'), 'w') as pr_f:
      # Mean average precision
      mAP = sum([a[1] for a in aps]) / len(aps)
      print('mAP: {}'.format(mAP))
      pr_f.write('mAP: {}\n'.format(mAP))

      # Individual precisions
      for k, ap in aps:
        print('\t{}: {}'.format(k, ap))
        pr_f.write('\t{}: {}\n'.format(k, ap))
