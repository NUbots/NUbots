#! /usr/bin/env python3

import math
import os
import re
import numpy as np
import tensorflow as tf
import time
import yaml

network_structure = [8, 8, 4]
max_height_delta = 0.0065
csv_data_cols = 12


def load_yaml_model(model):
    with open(model, 'r') as f:
        network = yaml.load(f)

    structure = []

    for layer in network:
        weights = None
        biases = None
        if 'weights' in layer:
            weights = np.array(layer['weights'], dtype=np.float32)
        if 'biases' in layer:
            biases = np.array(layer['biases'], dtype=np.float32)
        if weights is not None and biases is not None:
            structure.append({'weights': weights, 'biases': biases})

    return structure


# Convert the csv row into an example
def parse_row(line, truth):
    cols = tf.decode_csv(records=line, record_defaults=[[0.0]] * (csv_data_cols + 2))  # +2 for the foot heights
    data = tf.stack(cols[2:])
    foot_on_ground = tf.cast(
        tf.stack([
            cols[0] < (cols[1] + max_height_delta),
            cols[1] < (cols[0] + max_height_delta),
        ]), tf.float32
    )
    truth = tf.cond(
        tf.reduce_all(tf.equal(truth, -1)),
        lambda: tf.cast(foot_on_ground, tf.float32),
        lambda: tf.cast(truth, tf.float32),
    )
    truth = tf.reshape(tf.stack([truth, 1.0 - truth], axis=-1), [4])
    return data, truth


dataset = tf.data.Dataset.zip((
    tf.data.TextLineDataset('walk_load_datasets/long_walk.csv').skip(1),
    tf.data.Dataset.from_tensors([-1, -1]).repeat(),
))
dataset = dataset.map(parse_row)
dataset = dataset.repeat(1)
dataset = dataset.shuffle(1)
dataset = dataset.batch(1)
dataset_iterator = dataset.make_one_shot_iterator()

structure = load_yaml_model('best_vloss.yaml')

# Build the network using the provided structure
line = tf.placeholder(tf.string, [])
iterator = tf.data.Iterator.from_string_handle(line, dataset.output_types, dataset.output_shapes)
logits, labels = iterator.get_next()
logits = tf.reshape(logits, [-1, csv_data_cols])
for i, out_s in enumerate(network_structure):
    with tf.variable_scope('Layer{}'.format(i)):
        in_s = logits.get_shape()[1].value

        # Create weights and biases
        W = tf.get_variable('Weights', initializer=structure[i]['weights'], dtype=tf.float32)
        b = tf.get_variable('Biases', initializer=structure[i]['biases'], dtype=tf.float32)

        # Apply our weights and biases
        logits = tf.matmul(logits, W)
        logits = tf.add(logits, b)

        # Apply our activation function except on the last layer
        if i < len(network_structure) - 1:
            logits = tf.nn.selu(logits)
        else:
            logits = tf.reshape(
                tf.stack([tf.nn.softmax(logits=logits[:, 0:2]),
                          tf.nn.softmax(logits=logits[:, 2:4])]), [4]
            )

with tf.Session() as sess:
    sess.run(tf.global_variables_initializer())
    valid_handle = sess.run(dataset_iterator.string_handle('dataset'))

    start = time.time()
    confusion = np.zeros((2, 2), dtype=np.int)
    count = 0
    while True:
        try:
            out, truth = sess.run([tf.squeeze(logits), tf.squeeze(labels)], feed_dict={line: valid_handle})
            thresholded = [out[0] > out[1], out[0] < out[1], out[2] > out[3], out[2] < out[3]]

            print('Batch: {} -> Truth: {} Prediction: {} Thresholded: {}'.format(count, truth, out, thresholded))

            # Left
            if ((truth[0] == 1) and (thresholded[0] == 1) and (thresholded[1] == 0)):
                # TP
                confusion[0, 0] += 1
            elif ((truth[0] == 0) and (thresholded[0] == 0) and (thresholded[1] == 1)):
                # TN
                confusion[1, 1] += 1
            elif ((truth[0] == 1) and (thresholded[0] == 0) and (thresholded[1] == 1)):
                # FN
                confusion[1, 0] += 1
            elif ((truth[0] == 0) and (thresholded[0] == 1) and (thresholded[1] == 0)):
                # FP
                confusion[0, 1] += 1

            # Right
            if ((truth[2] == 1) and (thresholded[2] == 1) and (thresholded[2] == 0)):
                # TP
                confusion[0, 0] += 1
            elif ((truth[2] == 0) and (thresholded[2] == 0) and (thresholded[2] == 1)):
                # TN
                confusion[1, 1] += 1
            elif ((truth[2] == 1) and (thresholded[2] == 0) and (thresholded[2] == 1)):
                # FN
                confusion[1, 0] += 1
            elif ((truth[2] == 0) and (thresholded[2] == 1) and (thresholded[2] == 0)):
                # FP
                confusion[0, 1] += 1

            count += 1

        except tf.errors.OutOfRangeError:
            print(confusion)
            print('TPR: {}'.format(confusion[0, 0] / float(confusion[0, 0] + confusion[1, 0])))
            print('TNR: {}'.format(confusion[1, 1] / float(confusion[0, 1] + confusion[1, 1])))
            print('PPV: {}'.format(confusion[0, 0] / float(confusion[0, 0] + confusion[0, 1])))
            print('NPV: {}'.format(confusion[1, 1] / float(confusion[1, 0] + confusion[1, 1])))
            print('ACC: {}'.format((confusion[0, 0] + confusion[1, 1]) / float(confusion.sum())))
            print(
                'F1.: {}'.format((2 * confusion[0, 0]) / float(2 * confusion[0, 0] + confusion[0, 1] + confusion[1, 0]))
            )
            print(
                'MCC: {}'.format((confusion[0, 0] * confusion[1, 1] - confusion[0, 1] * confusion[1, 0]) / math.sqrt(
                    float(confusion[0, 0] + confusion[0, 1]) * float(confusion[1, 0] + confusion[1, 1]) *
                    float(confusion[0, 0] + confusion[1, 0]) * float(confusion[0, 1] + confusion[1, 1])
                ))
            )
            print('Testing took {:6f}s'.format(time.time() - start))

            break
