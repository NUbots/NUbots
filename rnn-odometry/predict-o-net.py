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
# test_data = test_data[:4000]
# test_targets = test_targets[:4000]

# Load model
model = load_model('models/model-20240524-195327')

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

# Inspect model
# layer = model.get_layer('lstm_1')
# layer_activations = layer.output[:, 0, :]
# print(f"shape of layer activations: {layer_activations.shape}")
# # print(f'Sample activations from the layer: {layer_activations[0][:5]}')
# print(f"Model weights: {model.get_weights()}")

# Check shapes
# print('test set shape:', test_targets)
print('prediction shape:', predictions.shape)
# print('prediction shape[0]:', predictions.shape[0])
# print('prediction shape[1]:', predictions.shape[1])
print('loaded target set shape:', test_targets.shape)
print('Sequenced target set shape: ', targets_sequenced.shape)

# Flatten predictions and targets (assuming all time steps are relevant)
# predictions_2d = predictions.reshape(-1, predictions.shape[-1])
# print('prediction 2d shape:', predictions_2d.shape)
# targets_sequenced_2d = targets_sequenced.reshape(-1, targets_sequenced.shape[-1])
# print('prediction 2d shape:', targets_sequenced_2d.shape)


# Evaluate
# mae = mean_absolute_error(predictions_2d, targets_sequenced_2d)
# r2 = r2_score(predictions_2d, targets_sequenced_2d)
# print(f"Mean Absolute Error (MAE): {mae:.2f}")  # Print MAE
# print(f"R-squared (R2) Score: {r2:.2f}")

# Plot and inspect
# plt.figure(figsize=(10, 6))
# plt.plot(predictions, 'r', label='Predictions')
# plt.plot(targets_sequenced, 'b', label='Targets')
# plt.legend()
# plt.show()
# num_channels = targets_sequenced.shape[3]
# plt.figure(figsize=(10, 6))
# # Plot each channel
# for i in range(num_channels):
#     plt.plot(targets_sequenced[0:50000, 0, i], label=f'Targets {i+1}')
#     plt.plot(predictions[0:50000, 0, i], label=f'Predictions {i+1}')
# # Add a legend
# plt.legend()
# plt.show()

# # Assuming your prediction set is called 'predictions' and target set is called 'targets'
# num_sequences = predictions.shape[0]
# sequence_length = predictions.shape[1]

# # Select a subset of sequences to plot (adjust the range as needed)
# sequence_indices = range(0, num_sequences, 10000)  # Plot every 100th sequence

# fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# for seq_idx in sequence_indices:
#     prediction = predictions[seq_idx]
#     target = targets_sequenced[seq_idx]

#     # Plot x coordinates
#     axs[0].plot(range(sequence_length), prediction[:, 0], label=f"Prediction {seq_idx} (X)")
#     axs[0].plot(range(sequence_length), target[:, 0], '--', label=f"Target {seq_idx} (X)")

#     # Plot y coordinates
#     axs[1].plot(range(sequence_length), prediction[:, 1], label=f"Prediction {seq_idx} (Y)")
#     axs[1].plot(range(sequence_length), target[:, 1], '--', label=f"Target {seq_idx} (Y)")

# axs[0].set_title("Predictions vs. Ground Truth (X Coordinates)")
# axs[1].set_title("Predictions vs. Ground Truth (Y Coordinates)")
# axs[1].set_xlabel("Time Step")

# axs[0].legend()
# axs[1].legend()

# plt.tight_layout()
# plt.show()

# single plot
num_time_steps = predictions.shape[1]

plt.figure(figsize=(10, 6))
plt.plot(predictions[:, 100, :], label=f'Predictions')
plt.plot(targets_sequenced[:, 100, :], label=f'Targets')
plt.legend()
plt.show()

# plt.figure(figsize=(10, 6))
# for i in range(num_time_steps):
#     plt.plot(predictions[:, i, :], label=f'Predictions')
#     plt.plot(targets_sequenced[:, i, :], label=f'Targets')
# plt.legend()
# plt.show()
