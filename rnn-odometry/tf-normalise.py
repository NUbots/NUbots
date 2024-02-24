#!/usr/bin/env python3

import os

import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf

# list of numpy arrays - NOTE: Change to loop through a directory given as an argument
# arrays = [np.array([1, 2, 3]), np.array([4, 5, 6]), np.array([7, 8, 9])]
print(os.getcwd())
array = np.load("processed-outputs/numpy/b/1/b-imu-1.npy")

# Convert the list of arrays to tensors
# tensors = [tf.convert_to_tensor(array, dtype=tf.float32) for array in arrays]
tensor = tf.convert_to_tensor(array, dtype=tf.float32)

# Concatenate all tensors into one
concatenated = tf.concat(tensor, axis=0)

# Calculate the min and max of the concatenated tensor
min_val = tf.reduce_min(concatenated)
max_val = tf.reduce_max(concatenated)

# TODO: Add buffer of some kind to the max and min values so that any new data can
# use the same normalisation.(like servos between 0-360 degrees)

# Normalize each tensor
normalized_tensor = [(tensor - min_val) / (max_val - min_val) for tensor in tensor]

print(normalized_tensor)
# normalized_arrays = []
# for tensor in normalized_tensor:
#     normalized_arrays.append(np.array(tensor))

normalized_arrays = np.array(normalized_tensor)

# Plot the original tensor as a scatter plot
plt.plot(array, color="red", alpha=0.7, label="Normalised")
plt.plot(normalized_arrays, color="blue", alpha=0.7, label="Unnormalised")
plt.legend()

plt.title("Normalised vs. Unnormalised Data")
plt.xlabel("Values")
plt.ylabel("Frequency")

plt.tight_layout()
plt.show()
