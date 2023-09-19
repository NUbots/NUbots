import os

import numpy as np
import tensorflow as tf

# list of numpy arrays - NOTE: Change to loop through a directory given as an argument
# arrays = [np.array([1, 2, 3]), np.array([4, 5, 6]), np.array([7, 8, 9])]
print(os.getcwd())
array = np.load("processed-outputs/numpy/input2.npy")

# Convert the list of arrays to tensors
# tensors = [tf.convert_to_tensor(array, dtype=tf.float32) for array in arrays]
tensors = tf.convert_to_tensor(array, dtype=tf.float32)

# Concatenate all tensors into one
concatenated = tf.concat(tensors, axis=0)

# Calculate the min and max of the concatenated tensor
min_val = tf.reduce_min(concatenated)
max_val = tf.reduce_max(concatenated)

# Normalize each tensor
normalized_tensors = [(tensor - min_val) / (max_val - min_val) for tensor in tensors]

print(normalized_tensors)
