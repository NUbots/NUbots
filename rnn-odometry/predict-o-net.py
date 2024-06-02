import joblib
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
from keras.models import load_model
from sklearn.metrics import mean_absolute_error, r2_score
from sklearn.preprocessing import MinMaxScaler

# Load data
test_data = np.load('datasets/input_data_train.npy')
test_targets = np.load('datasets/input_targets_train.npy')
# test_data = test_data[:4000]
# test_targets = test_targets[:4000]

# Load model
model = load_model('models/model-20240601-213604')

# Plot and inspect loaded data
# num_channels = test_data.shape[1]
# plt.figure(figsize=(10, 5))
# # Plot each channel
# for i in range(num_channels):
#     plt.plot(test_data[:20000, i], label=f'Channel {i+1}')
# # Add a legend
# plt.legend()
# plt.show()

system_sample_rate = 115
sequence_length = system_sample_rate * 1    # Look back 3 seconds
sequence_stride = 1                         # Shift one sequence_length at a time (rolling window)
sampling_rate = 1                           # Used for downsampling
batch_size = 115
#NOTE: Using return_sequences=True. sequence_length should be = axis 0 of array (total sequence length) (WRONG)
# Create test dataset

# sequence_length = test_data.shape[0]

test_dataset = tf.keras.utils.timeseries_dataset_from_array(
    data=test_data,
    targets=None,
    sequence_length=sequence_length,
    sequence_stride=sequence_stride,
    sampling_rate=sampling_rate,
    batch_size=batch_size
)
# target_dataset = tf.keras.utils.timeseries_dataset_from_array(
#     data=test_targets,
#     targets=None,
#     sequence_length=sequence_length,
#     sequence_stride=sequence_stride,
#     sampling_rate=sampling_rate,
#     batch_size=batch_size
# )
# test_dataset = tf.keras.utils.timeseries_dataset_from_array(
#     data=test_data,
#     targets=None,
#     sequence_length=sequence_length,
#     sequence_stride=sequence_stride,
#     sampling_rate=sampling_rate,
#     batch_size=batch_size
# )
# Assuming input_targets_train is your original ground truth data
targets_sequenced = np.array([test_targets[i: i + sequence_length] for i in range(test_targets.shape[0] - sequence_length + 1)])

# Predict
predictions = model.predict(test_dataset)

# Check shapes
# print('test set shape:', test_targets)
print('prediction shape:', predictions.shape)
# print('prediction shape[0]:', predictions.shape[0])
# print('prediction shape[1]:', predictions.shape[1])
print('loaded target set shape:', test_targets.shape)
print('Sequenced target set shape: ', targets_sequenced.shape)

# single plot
num_time_steps = predictions.shape[1]

plt.figure(figsize=(10, 6))
plt.plot(predictions[:, 100, :], label=f'Predictions')
plt.plot(targets_sequenced[:, 100, :], label=f'Targets')
plt.legend()
plt.show()
