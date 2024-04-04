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
model = load_model('models/model-20240330-152126')


system_sample_rate = 115
sequence_length = system_sample_rate * 2    # Look back 3 seconds
sequence_stride = 1                         # Shift one sequence_length at a time (rolling window)
sampling_rate = 1                           # Used for downsampling
batch_size = 1024

# Create test dataset
test_dataset = tf.keras.utils.timeseries_dataset_from_array(
    data=test_data,
    targets=test_targets,
    sequence_length=sequence_length,
    sequence_stride=sequence_stride,
    sampling_rate=sampling_rate,
    batch_size=batch_size
)

# Predict
predictions = model.predict(test_dataset)

# Check shapes
print('test set shape:', test_data.shape)
print('prediction shape:', predictions.shape)
print('prediction shape[0]:', predictions.shape[0])
print('prediction shape[1]:', predictions.shape[1])
print('target set shape:', test_targets.shape)

# Evaluate
test_targets = test_targets[:predictions.shape[0]]
mae = mean_absolute_error(test_targets, predictions)
r2 = r2_score(test_targets, predictions)
print(f"Mean Absolute Error (MAE): {mae:.2f}")  # Print MAE
print(f"R-squared (R2) Score: {r2:.2f}")

# # Plot and inspect
plt.figure(figsize=(10, 6))
plt.plot(predictions, 'r', label='Predictions')
plt.plot(test_targets, 'b', label='Targets')
plt.legend()
plt.show()
