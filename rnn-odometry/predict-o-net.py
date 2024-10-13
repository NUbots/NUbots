import joblib
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
from keras.models import load_model
from sklearn.metrics import mean_absolute_error, r2_score
from sklearn.preprocessing import MinMaxScaler


def partition_dataset(features, targets, sequence_length):
    inputs = []
    outputs = []

    for i in range(0, len(features) - sequence_length):
        window = features[i:i + sequence_length]
        next_value = targets[i + sequence_length]

        inputs.append(window)
        outputs.append(next_value)

    return np.array(inputs), np.array(outputs)


# Load data
test_data = np.load('datasets/input_data_train.npy')
test_targets = np.load('datasets/input_targets_train.npy')
# test_data = test_data[:4000]
# test_targets = test_targets[:4000]

# Load model
model = load_model('models/model-20241013-161411')

# Partition dataset
sequence_length = 64
test_data, test_targets = partition_dataset(test_data, test_targets, sequence_length)

# Predict
predictions = model.predict(test_data)

# Check shapes
print('prediction shape:', predictions.shape)
print('loaded target set shape:', test_targets.shape)
print('Sequenced target set shape: ', test_targets.shape)

# Calculate and print error metrics
mae = mean_absolute_error(test_targets, predictions)
r2 = r2_score(test_targets, predictions)
print('Mean Absolute Error:', mae)
print('R2 Score:', r2)


# Plot predictions vs targets
# plt.plot(predictions, label='predictions')
# plt.plot(test_targets, label='targets')
# plt.legend()
# plt.show()

# Plot predictions vs targets with x,y pairs from each set on separate sub plots.
num_channels = test_targets.shape[1]
plt.figure(figsize=(10, 5 + num_channels * 2))

# Plot each channel
for i in range(num_channels):
    plt.subplot(num_channels + 1, 1, i + 1)
    plt.plot(predictions[:, i], label='predictions')
    plt.plot(test_targets[:, i], label='targets')
    plt.legend()

# Plot test_data on its own subplot
plt.subplot(num_channels + 1, 1, num_channels + 1)
for i in range(test_data.shape[2]):
    plt.plot(test_data[:, -1, i], label=f'test_data_channel_{i}')
plt.legend()

plt.show()
