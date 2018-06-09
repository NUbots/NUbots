#! /usr/bin/env python3

import math
import os
import re
import numpy as np
import tensorflow as tf
import time
import yaml

max_height_delta = 0.0065
network_structure = [8, 8, 4]
batch_size = 50
epochs = 5000
shuffle_size = 10000
csv_data_cols = 12
learning_rate = 0.0001


def save_yaml_model(sess, output_path, global_step, loss, variables=None, name=None):

    # Run tf to get all our variables
    variables = {v.name: sess.run(v) for v in tf.trainable_variables()} if variables is None else variables
    output = []
    output.append({'loss': loss})

    # So we know when to move to the next list
    conv = -1
    layer = -1

    # Convert the keys into useful data
    items = []
    for k, v in variables.items():
        info = re.match(r'Layer(\d+)/(Weights|Biases):0', k)
        if info:
            items.append(((int(info.group(1)), info.group(2).lower()), v))

    # Sorted so we see earlier layers first
    for k, v in sorted(items):
        l = k[0]
        var = k[1]

        # If we change layer add a new object
        if l != layer:
            output.append({})
            layer = l

        output[layer][var] = v.tolist()

    # Print as yaml
    os.makedirs(os.path.join(output_path, 'yaml_models'), exist_ok=True)
    output_name = 'model_{}.yaml'.format(global_step) if name is None else '{}.yaml'.format(name)
    with open(os.path.join(output_path, 'yaml_models', output_name), 'w') as f:
        f.write(yaml.dump(output, width=120))


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


datasets = []

# Negative files
for f in [
        'walk_load_datasets/laying_back.csv',
        'walk_load_datasets/laying_front.csv',
        'walk_load_datasets/picked_up.csv',
        'walk_load_datasets/walking_picked_up.csv',
        'walk_load_datasets/walking_laying_back.csv',
        'walk_load_datasets/walking_laying_front.csv',
]:
    datasets.append(
        tf.data.Dataset.zip((
            tf.data.TextLineDataset(f).skip(1),
            tf.data.Dataset.from_tensors([0, 0]).repeat(),
        ))
    )

# Positive files
for f in [
        'walk_load_datasets/standing.csv',
]:
    datasets.append(
        tf.data.Dataset.zip((
            tf.data.TextLineDataset(f).skip(1),
            tf.data.Dataset.from_tensors([1, 1]).repeat(),
        ))
    )

# Files that have the feet changing based on the walk
for f in [
        'walk_load_datasets/walking.csv',
        'walk_load_datasets/walking2.csv',
        'walk_load_datasets/walking3.csv',
        'walk_load_datasets/walking4.csv',
]:
    datasets.append(
        tf.data.Dataset.zip((
            tf.data.TextLineDataset(f).skip(1),
            tf.data.Dataset.from_tensors([-1, -1]).repeat(),
        ))
    )

dataset = datasets[0]
for ds in datasets[1:]:
    dataset = dataset.concatenate(ds)

dataset = dataset.map(parse_row)
dataset = dataset.repeat(epochs)
dataset = dataset.shuffle(shuffle_size)
dataset = dataset.batch(batch_size)
train_iterator = dataset.make_initializable_iterator()

valid_dataset = tf.data.Dataset.zip((
    tf.data.TextLineDataset('walk_load_datasets/long_walk.csv').skip(1),
    tf.data.Dataset.from_tensors([-1, -1]),
))
valid_dataset = valid_dataset.map(parse_row)
valid_dataset = valid_dataset.repeat(epochs)
valid_dataset = valid_dataset.shuffle(shuffle_size)
valid_dataset = valid_dataset.batch(batch_size)
valid_iterator = valid_dataset.make_initializable_iterator()

# Build the network using the provided structure
line = tf.placeholder(tf.string, [])
iterator = tf.data.Iterator.from_string_handle(line, dataset.output_types, dataset.output_shapes)
logits, labels = iterator.get_next()
logits = tf.reshape(logits, [-1, csv_data_cols])
for i, out_s in enumerate(network_structure):
    with tf.variable_scope('Layer{}'.format(i)):
        in_s = logits.get_shape()[1].value

        # Create weights and biases
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
        logits = tf.matmul(logits, W)
        logits = tf.add(logits, b)

        # Apply our activation function except on the last layer
        if i < len(network_structure) - 1:
            logits = tf.nn.selu(logits)

# Optimiser
global_step = tf.Variable(0, dtype=tf.int32, trainable=False, name='global_step')
loss = tf.reduce_mean(
    tf.nn.softmax_cross_entropy_with_logits_v2(logits=logits[:, 0:2], labels=labels[:, 0:2], dim=1) +
    tf.nn.softmax_cross_entropy_with_logits_v2(logits=logits[:, 2:4], labels=labels[:, 2:4], dim=1)
)

optimiser = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(loss, global_step=global_step)

with tf.Session() as sess:
    sess.run(tf.global_variables_initializer())
    sess.run([train_iterator.initializer, valid_iterator.initializer])
    train_handle, valid_handle = sess.run([
        train_iterator.string_handle('train_dataset'),
        valid_iterator.string_handle('valid_dataset'),
    ])

    best_loss = np.inf
    best_model = None
    best_step = -1
    tloss = 0
    duration = 0
    start = time.time()
    while True:
        try:
            begin = time.time()
            _, l, = sess.run([optimiser, loss], feed_dict={line: train_handle})
            duration += time.time() - begin
            step = tf.train.global_step(sess, global_step)
            tloss += l

            if step and (step % 100) == 0:
                tloss /= 100
                duration /= 100
                print('Batch {:5d}: duration (avg) {:6f}, train loss (avg) {:6f}'.format(step, duration, tloss))
                tloss = 0
                duration = 0

            if step and (step % 1000) == 0:
                vloss = 0
                count = 0
                while True:
                    try:
                        l = sess.run(loss, feed_dict={line: valid_handle})
                        vloss += l
                        count += 1
                    except tf.errors.OutOfRangeError:
                        vloss /= count

                        if vloss < best_loss:
                            best_vloss = loss
                            save_yaml_model(
                                sess, '.', tf.train.global_step(sess, global_step), vloss, name='best_vloss'
                            )

                        print('Batch {:5d}: valid loss (avg) {:6f}'.format(step, vloss))
                        sess.run(valid_iterator.initializer)
                        valid_handle = sess.run(valid_iterator.string_handle('valid_dataset'))
                        break

            if step and (step % 5000) == 0:
                # Save our model in yaml format
                save_yaml_model(sess, '.', tf.train.global_step(sess, global_step), l)

        except tf.errors.OutOfRangeError:
            print('Training done')
            print('Training took {:6f}s'.format(time.time() - start))

            # Save our final model
            save_yaml_model(sess, '.', tf.train.global_step(sess, global_step), l)
            break
