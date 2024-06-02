#!/usr/bin/env python3

import argparse
import datetime
import os
import subprocess

import joblib
import keras
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
from scipy.ndimage import gaussian_filter1d
from sklearn.preprocessing import MinMaxScaler


def main():

    def convert_to_relative(data):
        """
        This function takes a dataset of world coordinates and converts it to relative positions
        based on the starting position (assumed to be the first index).

        Args:
            data: A NumPy array of shape (num_datapoints, 3) representing world coordinates.

        Returns:
            A NumPy array of shape (num_datapoints, 3) representing relative positions.
        """

        # Get the starting position (assuming first index)
        starting_position = data[0]

        # Calculate relative positions for all data points (excluding starting position)
        relative_positions = data - starting_position

        return relative_positions

    def gaussian_smooth(data, window_size):
        "Takes an array and separately applies gaussian filter to each dimension"

        # Output array
        smoothed_data = np.empty_like(data)

        # Apply gaussian filter to each dimension
        for i in range(data.shape[1]):
            smoothed_data[:, i] = gaussian_filter1d(data[:, i], window_size, axis=0)

        # print shapes to verify i didn't f up
        # print("Original data shape: ", data.shape)
        # print("Smoothed data shape: ", smoothed_data.shape)
        return smoothed_data

    # numpy arrays
    first_file = 7
    num_files = 22  # Number of files to load
    prefix = "s"  # s for servo, h for head, a for arm
    imu = []
    servos = []
    truth_all = []

    for i in range(first_file, num_files):
        imu.append(np.load(f"processed-outputs/numpy/{prefix}/{i}/{prefix}-imu-{i}.npy"))
        servos.append(np.load(f"processed-outputs/numpy/{prefix}/{i}/{prefix}-servos-{i}.npy"))
        truth_all.append(np.load(f"processed-outputs/numpy/{prefix}/{i}/{prefix}-truth-{i}.npy"))

    # Need to do the relative conversions here
    # Use the convert_to_relative function to convert the truth data to relative positions
    truth_all = [convert_to_relative(truth) for truth in truth_all]

    # Smoothing should be done here
    # Loop through each truth array and smooth
    # truth_all = [gaussian_smooth(truth, 50) for truth in truth_all]

    # Loop through each array and concatenate into a numpy array
    imu_joined = np.concatenate(imu, axis=0)
    servos_joined = np.concatenate(servos, axis=0)
    truth_all_joined = np.concatenate(truth_all, axis=0)

    # Print the shape of the joined arrays
    print("IMU s joined: ", imu_joined.shape)
    print("Servos s joined: ", servos_joined.shape)
    print("Truth s joined: ", truth_all_joined.shape)

    # Slice out the arm and head servos
    servos_joined = servos_joined[:, 6:18]

    # Plot and inspect each joined array
    # num_channels = truth_all_joined.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     plt.plot(truth_all_joined[0:100000, i], label=f'servo {i+1}')
    # # Add a legend
    # # plt.ylim(np.min(imu), np.max(imu))
    # plt.autoscale(enable=True, axis="both")
    # plt.legend()
    # plt.show()

    # NOTE: Using only position for first tests to see what happens. May add in heading later.
    # Filter the truth down to just position -  this is the last column of the homogenous transform matrix
    # NOTE: 17/04/2024 - Due to fluctuations in the z component, trying with just x, y
    # NOTE: Remember to reshape if adding or removing features

    # slice out the x and y positions (9:11) of the truth array
    truth_joined_sliced = truth_all_joined[:, 9:11]

    # Plot and inspect after slicing
    # num_channels = truth_joined_sliced.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     plt.plot(truth_joined_sliced[100000:, i], label=f'Position {i+1}')
    # # Add a legend
    # plt.legend()
    # plt.show()


    # Join the data
    joined_data = np.concatenate([imu_joined, servos_joined, truth_joined_sliced], axis=1)
    print("Total joined data shape: ", joined_data.shape)

    # Split the training data into training, validation and test sets
    training_size = 0.5
    validate_size = 0.2
    num_time_steps = joined_data.shape[0]

    num_train_max_idx = int(np.floor(num_time_steps * training_size))
    num_val_max_idx = int(np.floor(num_time_steps * (training_size + validate_size)))

    # import pdb
    # pdb.set_trace()

    train_arr = joined_data[:num_train_max_idx]
    validate_arr = joined_data[num_train_max_idx: num_val_max_idx]
    test_arr = joined_data[num_val_max_idx:]

    # import pdb
    # pdb.set_trace()

    # array sizes
    # num_train = train_arr.size
    # num_val = validate_arr.size

    #### Normalisation  ####
    ## Standardise ##
    # NOTE: mean and std from training dataset is used to standardise
    # all of the datasets to prevent information leakage.
    # mean and std from the training run will need to be used in production for de-standardise the predictions.
    mean = train_arr.mean(axis=0)
    std = train_arr.std(axis=0)
    print("mean: ", mean)
    print("std: ", std)

    train_arr = (train_arr - mean) / std
    validate_arr = (validate_arr - mean) / std
    test_arr = (test_arr - mean) / std
    # # # #

    print(f"Training set size: {train_arr.shape}")
    print("Training set min:", np.min(train_arr))
    print("Training set max:", np.max(train_arr))
    print(f"Validation set size: {validate_arr.shape}")
    print(f"Test set size: {test_arr.shape}")

    # # clip the outliers in the data
    # train_arr_clipped = np.clip(train_arr, -4, 4)
    # validate_arr_clipped = np.clip(validate_arr, -4, 4)
    # test_arr_clipped = np.clip(test_arr, -4, 4)

    ## Min/Max Scaling ##
    # scaler = MinMaxScaler(feature_range=(-1, 1))
    # # Fit scaler to training data only
    # scaler.fit(train_arr)
    # # Transform the data
    # train_arr_scaled = scaler.transform(train_arr)
    # validate_arr_scaled = scaler.transform(validate_arr)
    # test_arr_scaled = scaler.transform(test_arr)

    # Plot and inspect after normalising
    # num_channels = test_arr.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     plt.plot(test_arr[0:100000, i], label=f'Channel {i+1}')
    # # Add a legend
    # plt.legend()
    # plt.show()

    # NOTE: Remember to reshape if adding or removing features

    # Split into data and targets
    # Training
    input_data_train = train_arr[:, :18]  # imu and servos
    input_targets_train = train_arr[:, 18:]  # truth
    # Convert sliced targets to relative position
    # input_targets_train = convert_to_relative(input_targets_train)

    # Validation
    input_data_validate = validate_arr[:, :18]  # imu and servos
    input_targets_validate = validate_arr[:, 18:]  # truth
    # Convert sliced targets to relative position
    # input_targets_validate = convert_to_relative(input_targets_validate)

    # Testing
    input_data_test= test_arr[:, :18]  # imu and servos
    input_targets_test = test_arr[:, 18:]  # truth
    # Convert sliced targets to relative position
    # input_targets_test = convert_to_relative(input_targets_test)

    # Plot and inspect after splitting
    # num_channels = input_targets_train.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     plt.plot(input_targets_train[0:100000, i], label=f'Target {i+1}')
    # # Add a legend
    # plt.legend()
    # plt.show()


    # print dataset shapes
    print(f"input_data_train: {input_data_train.shape}")
    print(f"input_targets_train: {input_targets_train.shape}")
    print(f"input_data_validate: {input_data_validate.shape}")
    print(f"input_targets_validate: {input_targets_validate.shape}")
    print(f"input_data_test: {input_data_test.shape}")
    print(f"input_targets_test: {input_targets_test.shape}")

     # Save the datasets (should break the dataset creation to another module eventually)
    np.save('datasets/input_data_train.npy',input_data_train)
    np.save('datasets/input_targets_train.npy',input_targets_train)
    np.save('datasets/input_data_validate.npy', input_data_validate)
    np.save('datasets/input_targets_validate.npy', input_targets_validate)
    np.save('datasets/input_data_test.npy', input_data_test)
    np.save('datasets/input_targets_test.npy', input_targets_test)

    # Plot and inspect
    # num_channels = train_arr_scaled.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     if i < 6:
    #         plt.plot(train_arr_scaled[0:50000, i], label=f'Imu {i+1}', color='g')
    #     elif i < 18:
    #         plt.plot(train_arr_scaled[0:50000, i], label=f'Servos {i+1}', color='b')
    #     else:
    #         plt.plot(train_arr_scaled[0:50000, i], label=f'Truth {i+1}', color='r')
    # # Add a legend
    # plt.legend()
    # plt.show()

    # NOTE: Samples are roughly 115/sec
    system_sample_rate = 115
    sequence_length = system_sample_rate * 1   # Look back n seconds (system_sample_rate * n). system_sample_rate was roughly calculated at 115/sec
    sequence_stride = 1                         # Shift one sequence_length at a time (rolling window)
    sampling_rate = 1                           # Used for downsampling
    batch_size = 115                         # Number of samples per gradient update (original: 64, seemed better?: 512)

    # Sequence lengths (for return sequences)
    # train_seq_length = input_data_train.shape[0]
    # val_seq_length = input_data_validate.shape[0]
    # test_seq_length = input_data_test.shape[0]

    # Reshape target data due to return_sequences=True (prediction per timestep)


    train_dataset_features = tf.keras.utils.timeseries_dataset_from_array(
        data=input_data_train,
        targets=None,
        sequence_length=sequence_length,
        sequence_stride=sequence_stride,
        sampling_rate=sampling_rate,
        batch_size=batch_size
    )

    train_dataset_targets = tf.keras.utils.timeseries_dataset_from_array(
        data=input_targets_train,
        targets=None,
        sequence_length=sequence_length,
        sequence_stride=sequence_stride,
        sampling_rate=sampling_rate,
        batch_size=batch_size
    )
    train_dataset = tf.data.Dataset.zip((train_dataset_features, train_dataset_targets))

    validate_dataset_features = tf.keras.utils.timeseries_dataset_from_array(
        data=input_data_validate,
        targets=None,
        sequence_length=sequence_length,
        sequence_stride=sequence_stride,
        sampling_rate=sampling_rate,
        batch_size=batch_size
    )

    validate_dataset_targets = tf.keras.utils.timeseries_dataset_from_array(
        data=input_targets_validate,
        targets=None,
        sequence_length=sequence_length,
        sequence_stride=sequence_stride,
        sampling_rate=sampling_rate,
        batch_size=batch_size
    )
    validate_dataset = tf.data.Dataset.zip((validate_dataset_features, validate_dataset_targets))

    test_dataset_features = tf.keras.utils.timeseries_dataset_from_array(
        data=input_data_test,
        targets=None,
        sequence_length=sequence_length,
        sequence_stride=sequence_stride,
        sampling_rate=sampling_rate,
        batch_size=batch_size
    )

    test_dataset_targets = tf.keras.utils.timeseries_dataset_from_array(
        data=input_targets_test,
        targets=None,
        sequence_length=sequence_length,
        sequence_stride=sequence_stride,
        sampling_rate=sampling_rate,
        batch_size=batch_size
    )
    test_dataset = tf.data.Dataset.zip((test_dataset_features, test_dataset_targets))

    # Model parameters
    learning_rate = 0.00096   # Controls how much to change the model in response to error.
    epochs = 1500

    # Scheduler function keeps the initial learning rate for the first ten epochs
    # and decreases it exponentially after that. Uncomment and add lr_callback to model.fit callbacks array
    # def scheduler(epoch, lr):
    #     if epoch < 10:
    #         return lr
    #     else:
    #         return lr * tf.exp(-0.1)
    # lr_callback = keras.callbacks.LearningRateScheduler(scheduler)

    # ** Loss functions **
    loss_function = keras.losses.MeanAbsoluteError()
    # loss_function = keras.losses.MeanSquaredError()
    # loss_function = keras.losses.LogCosh()
    # loss_function = quantile_loss????
    # loss_function = keras.losses.Huber(delta=0.5)

    # ** Optimizers **
    # LR schedules
    size_of_dataset = input_data_train.shape[0]
    decay_to_epoch = 50                                         # Number of epochs for learning rate to decay over before it resets
    steps_per_epoch = size_of_dataset // batch_size              # Calculate the number of steps per epoch
    decay_over_steps = decay_to_epoch * steps_per_epoch         # Calculate the number of steps to decay over (scheduler takes the values in steps)
    print(f"Decay to epoch: {decay_to_epoch}")
    print(f"Number of steps to decay over before LR resets: {decay_over_steps}")
    lr_schedule = keras.optimizers.schedules.CosineDecayRestarts(initial_learning_rate=learning_rate, first_decay_steps=decay_over_steps, t_mul=1.00, m_mul=1.08, alpha=0.000001)

    # standard optimisers
    # optimizer = keras.optimizers.Adam(learning_rate=lr_schedule, beta_1=0.90)
    # optimizer = keras.optimizers.AdamW(learning_rate=lr_schedule)
    # optimizer = keras.optimizers.AdamW(learning_rate=learning_rate)

    optimizer=keras.optimizers.Adadelta(learning_rate=lr_schedule)
    # optimizer=keras.optimizers.Adadelta(learning_rate=learning_rate)
    # optimizer = keras.optimizers.SGD(learning_rate=lr_schedule, momentum=0.1)

    # Scheduled
    # lr_schedule = keras.optimizers.schedules.ExponentialDecay(
    #     initial_learning_rate=0.5,
    #     decay_steps=500000,
    #     decay_rate=0.0001)
    # lr_schedule = keras.optimizers.schedules.CosineDecay(
    #     initial_learning_rate=0.0005,
    #     decay_steps=5000000,
    #     alpha=0.000001
    # )

    # optimizer = keras.optimizers.RMSprop(learning_rate=learning_rate)

    # ** Activation functions **
    # activation = tf.keras.activations.leaky_relu(negative_slope=0.01)
    # activation = tf.keras.activations.relu(negative_slope=0.0, max_value=None, threshold=0.0)

    # Tensorboard
    timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    log_dir = "logs/fit/" + timestamp
    tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir=log_dir, histogram_freq=1)

    # Model Layers
    inputs = keras.layers.Input(shape=(sequence_length, input_data_train.shape[1]))
    dropout = keras.layers.Dropout(rate=0.47)(inputs)
    lstm = keras.layers.LSTM(150, kernel_initializer=keras.initializers.GlorotNormal(), kernel_regularizer=keras.regularizers.L1L2(l1=0.0015, l2=0.015), return_sequences=True)(dropout)    # 32 originally
    normalise = keras.layers.LayerNormalization()(lstm)

    dropout2 = keras.layers.Dropout(rate=0.47)(normalise)
    lstm2 = keras.layers.LSTM(150, kernel_initializer=keras.initializers.GlorotNormal(), kernel_regularizer=keras.regularizers.L1L2(l1=0.0015, l2=0.015), return_sequences=True)(dropout2)    # 32 originally
    normalise2 = keras.layers.LayerNormalization()(lstm2)

    dropout3 = keras.layers.Dropout(rate=0.47)(normalise2)
    lstm3 = keras.layers.LSTM(150, kernel_initializer=keras.initializers.GlorotNormal(), kernel_regularizer=keras.regularizers.L1L2(l1=0.0015, l2=0.015), return_sequences=True)(dropout3)    # 32 originally
    normalise3 = keras.layers.LayerNormalization()(lstm3)

    # Apply attention layer that considers lstm outputs
    attention = keras.layers.Attention()([normalise, normalise3])

    # lstm4 = keras.layers.LSTM(80, kernel_initializer=keras.initializers.HeNormal(), kernel_regularizer=keras.regularizers.L1L2(l1=0.00019, l2=0.0009), return_sequences=True)(attention)    # 32 originally
    # normalise4 = keras.layers.LayerNormalization()(lstm4)

    # # Compute dot product between attention weights and last LSTM layer
    # context_vector = keras.layers.Dot(axes=(1, 1))([attention, normalise3])

    # lstm4 = keras.layers.LSTM(80, kernel_initializer=keras.initializers.HeNormal(), kernel_regularizer=keras.regularizers.L1L2(l1=0.00019, l2=0.0009), return_sequences=True)(attention)    # 32 originally
    # normalise4 = keras.layers.LayerNormalization()(lstm4)
    # dropout4 = keras.layers.Dropout(rate=0.35)(normalise4)
    # NOTE: Changed dense layer units to 2 due to removing z component
    # dropout4 = keras.layers.Dropout(rate=0.2)(normalise3)
    # dense1 = keras.layers.Dense(32, kernel_regularizer=keras.regularizers.L1L2(l1=0.0001, l2=0.002))(normalise3)
    dense2 = keras.layers.TimeDistributed(keras.layers.Dense(2, kernel_regularizer=keras.regularizers.L1L2(l1=0.00001, l2=0.0002)))(attention)   # Target shape[1] is 3
    model = keras.Model(inputs=inputs, outputs=dense2)
    model.compile(optimizer=optimizer, loss=loss_function, metrics=["mse"])
    model.summary()

    Examples
    lstm = keras.layers.Bidirectional(LSTM(200, return_sequences=True, recurrent_regularizer=keras.regularizers.L1L2(l1=0.0002, l2=0.006)))(dropout)
    lstm2 = keras.layers.LSTM(50, return_sequences=True, kernel_initializer=keras.initializers.HeUniform(), kernel_regularizer=keras.regularizers.L1L2(l1=0.00001, l2=0.0002), recurrent_regularizer=keras.regularizers.L1L2(l1=0.00001, l2=0.0002))(dropout)    # 32 originally
    dropout2 = keras.layers.Dropout(rate=0.2)(lstm2)

    lstm3 = keras.layers.LSTM(10, return_sequences=False, kernel_initializer=keras.initializers.HeUniform(), kernel_regularizer=keras.regularizers.L1L2(l1=0.00001, l2=0.0002), recurrent_regularizer=keras.regularizers.L1L2(l1=0.00001, l2=0.0002))(dropout2)    # 32 originally
    dropout3 = keras.layers.Dropout(rate=0.2)(lstm3)

    model.fit(
        train_dataset,
        validation_data=validate_dataset,
        epochs=epochs,
        callbacks=[tensorboard_callback]
    )

    # Note add back the model save
    model.save("models/model-" + timestamp)
    # Save the scaler object
    # joblib.dump(scaler, 'scalers/scaler-' + timestamp)


if __name__ == "__main__":
    main()
