import joblib
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
from keras.models import load_model
from sklearn.metrics import mean_absolute_error, r2_score
from sklearn.preprocessing import MinMaxScaler


def partition_dataset(features, targets, window_size):
    inputs = []
    outputs = []

    for i in range(0, len(features) - window_size):
        window = features[i:i + window_size]
        next_value = targets[i + window_size]

        inputs.append(window)
        outputs.append(next_value)

    return np.array(inputs), np.array(outputs)


# Load data
test_data = np.load('datasets/input_data_test.npy')
test_targets = np.load('datasets/input_targets_test.npy')
# test_data = test_data[:4000]
# test_targets = test_targets[:4000]

# Load model
model = load_model('models/model-20240807-203901')

# Partition dataset
window_size = 100
test_data, test_targets = partition_dataset(test_data, test_targets, window_size)

# Predict
predictions = model.predict(test_data)

# Check shapes
print('prediction shape:', predictions.shape)
print('loaded target set shape:', test_targets.shape)
print('Sequenced target set shape: ', test_targets.shape)

# Plot predictions vs targets
plt.plot(predictions, label='predictions')
plt.plot(test_targets, label='targets')
plt.legend()
plt.show()
