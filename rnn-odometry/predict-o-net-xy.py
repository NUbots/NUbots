import joblib
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
from keras.models import load_model
from sklearn.metrics import mean_absolute_error, r2_score
from sklearn.preprocessing import MinMaxScaler

# Load data
test_data = np.load('datasets/input_data_test.npy')
test_targets = np.load('datasets/input_targets_test.npy')

# Load model
model = load_model('models/model-20240613-180738')

system_sample_rate = 115
sequence_length = system_sample_rate * 1
sequence_stride = 1
sampling_rate = 1
batch_size = 115

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

test_dataset = tf.data.Dataset.zip((test_dataset_features, (test_dataset_targets_x, test_dataset_targets_y)))

# Predict
predictions = model.predict(test_dataset)

# Convert predictions to numpy array
predictions = np.array(predictions)

# Check shapes
print('prediction shape:', predictions.shape)
print('loaded target set shape:', test_targets.shape)

# single plot
num_time_steps = predictions.shape[1]

plt.figure(figsize=(10, 6))
plt.plot(predictions[0, :, 0], label=f'Predictions x')
plt.plot(predictions[0, :, 1], label=f'Predictions y')
plt.plot(test_targets_x[:num_time_steps], label=f'Targets x')
plt.plot(test_targets_y[:num_time_steps], label=f'Targets y')
plt.legend()
plt.show()
