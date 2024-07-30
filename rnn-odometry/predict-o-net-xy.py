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

# Load model
model = load_model('models/model-20240730-193204')

system_sample_rate = 115
sequence_length = system_sample_rate * 1
sequence_stride = 1
sampling_rate = 1
batch_size = 65

test_dataset_features = tf.keras.utils.timeseries_dataset_from_array(
    data=test_data,
    targets=None,
    sequence_length=sequence_length,
    sequence_stride=sequence_stride,
    sampling_rate=sampling_rate,
    batch_size=batch_size
)

test_targets_x = test_targets[:, 0]
test_targets_y = test_targets[:, 1]

test_dataset_targets_x = tf.keras.utils.timeseries_dataset_from_array(
    data=test_targets_x,
    targets=None,
    sequence_length=sequence_length,
    sequence_stride=sequence_stride,
    sampling_rate=sampling_rate,
    batch_size=batch_size
)

test_dataset_targets_y = tf.keras.utils.timeseries_dataset_from_array(
    data=test_targets_y,
    targets=None,
    sequence_length=sequence_length,
    sequence_stride=sequence_stride,
    sampling_rate=sampling_rate,
    batch_size=batch_size
)

# test_dataset = tf.data.Dataset.zip((test_dataset_features, (test_dataset_targets_x, test_dataset_targets_y)))
test_dataset = tf.data.Dataset.zip((test_dataset_features, test_dataset_targets_y))

# Predict
predictions = model.predict(test_dataset)

# Convert predictions to numpy array
predictions = np.array(predictions)

# Check shapes
print('prediction shape:', predictions.shape)
print('loaded target set shape x:', test_targets_x.shape)
print('loaded target set shape y:', test_targets_y.shape)

# Adjust the plotting code to match the correct shapes
# plt.figure(figsize=(10, 6))
# # Adjust the index for predictions to match the correct dimension
# plt.plot(predictions[0, :, 0], label='Predictions x')  # Adjusted for correct indexing
# plt.plot(predictions[1, :, 0], label='Predictions y')  # Adjusted for correct indexing
# # Adjust the slicing of targets to match the length of predictions
# plt.plot(test_targets_x[:predictions.shape[1]], label='Targets x')  # Adjusted for correct length
# plt.plot(test_targets_y[:predictions.shape[1]], label='Targets y')  # Adjusted for correct length
# plt.legend()
# plt.show()

# Single prediction
plt.figure(figsize=(10, 6))
# Directly use the predictions array without additional indexing
plt.plot(predictions[:, -1], label='Predictions')  # Assuming you want to plot the last prediction for each sequence
plt.plot(test_targets_y[:predictions.shape[0]], label='Targets y')  # Adjusted for correct length, matching the number of sequences
plt.legend()
plt.show()
