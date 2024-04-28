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
model = load_model('models/model-20240428-184520')


# Load scaler
# scaler = joblib.load("scalers/scaler-20240425-095029")
# test_datascaler.fit()

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
sequence_length = system_sample_rate * 2    # Look back 3 seconds
sequence_stride = 1                         # Shift one sequence_length at a time (rolling window)
sampling_rate = 1                           # Used for downsampling
batch_size = 500
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
target_dataset = tf.keras.utils.timeseries_dataset_from_array(
    data=test_targets,
    targets=None,
    sequence_length=sequence_length,
    sequence_stride=sequence_stride,
    sampling_rate=sampling_rate,
    batch_size=batch_size
)
# test_dataset = tf.keras.utils.timeseries_dataset_from_array(
#     data=test_data,
#     targets=None,
#     sequence_length=sequence_length,
#     sequence_stride=sequence_stride,
#     sampling_rate=sampling_rate,
#     batch_size=batch_size
# )

# # Predict
predictions = model.predict(test_dataset)

# Inspect model
# layer = model.get_layer('lstm_1')
# layer_activations = layer.output[:, 0, :]
# print(f"shape of layer activations: {layer_activations.shape}")
# # print(f'Sample activations from the layer: {layer_activations[0][:5]}')
# print(f"Model weights: {model.get_weights()}")

# Check shapes
# print('test set shape:', test_targets)
print('prediction shape:', predictions.shape)
print('prediction shape[0]:', predictions.shape[0])
print('prediction shape[1]:', predictions.shape[1])
print('loaded target set shape:', test_targets.shape)

# Flatten predictions and targets (assuming all time steps are relevant)
predictions_2d = predictions.reshape(-1, predictions.shape[-1])
# targets_2d = np.concatenate([batch for batch in target_dataset.unbatch()], axis=0).reshape(-1, 2)
print('prediction 2d shape:', predictions_2d.shape)
# print('target shape sliced:', targets_2d.shape)


# Evaluate
# test_targets = test_targets[:predictions.shape[0]]
mae = mean_absolute_error(targets_2d, predictions_2d)
r2 = r2_score(targets_2d, predictions_2d)
print(f"Mean Absolute Error (MAE): {mae:.2f}")  # Print MAE
print(f"R-squared (R2) Score: {r2:.2f}")

# # Plot and inspect
# plt.figure(figsize=(10, 6))
# plt.plot(predictions_2d, 'r', label='Predictions')
# plt.plot(targets_2d, 'b', label='Targets')
# plt.legend()
# plt.show()
num_channels = targets_2d.shape[1]
plt.figure(figsize=(10, 6))
# Plot each channel
for i in range(num_channels):
    plt.plot(targets_2d[0:50000, i], label=f'Targets {i+1}')
    plt.plot(predictions_2d[0:50000, i], label=f'Predictions {i+1}')
# Add a legend
plt.legend()
plt.show()
