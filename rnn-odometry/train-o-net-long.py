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
        print("Original data shape: ", data.shape)
        print("Smoothed data shape: ", smoothed_data.shape)
        return smoothed_data

    # numpy arrays
    imu_long = np.load("processed-outputs/numpy/long/1/long-imu-1.npy")
    servos_long = np.load("processed-outputs/numpy/long/1/long-servos-1.npy")
    truth_all_long = np.load("processed-outputs/numpy/long/1/long-truth-1.npy")
    # More data
    imu_long_2 = np.load("processed-outputs/numpy/long/2/long-imu-2.npy")
    servos_long_2 = np.load("processed-outputs/numpy/long/2/long-servos-2.npy")
    truth_all_long_2 = np.load("processed-outputs/numpy/long/2/long-truth-2.npy")
    # Even more data
    imu_long_3 = np.load("processed-outputs/numpy/long/3/long-imu-3.npy")
    servos_long_3 = np.load("processed-outputs/numpy/long/3/long-servos-3.npy")
    truth_all_long_3 = np.load("processed-outputs/numpy/long/3/long-truth-3.npy")
    # Even more data
    imu_long_4 = np.load("processed-outputs/numpy/long/4/long-imu-4.npy")
    servos_long_4 = np.load("processed-outputs/numpy/long/4/long-servos-4.npy")
    truth_all_long_4 = np.load("processed-outputs/numpy/long/4/long-truth-4.npy")

    # Slice out the arm and head servos
    # NOTE: Remember to reshape if adding or removing features
    servos_long = servos_long[:, 6:18]
    servos_long_2 = servos_long_2[:, 6:18]
    servos_long_3 = servos_long_3[:, 6:18]
    servos_long_4 = servos_long_4[:, 6:18]

    # tstamps = np.load('processed-outputs/numpy/long/1/long-tstamps-1.npy')
    print("IMU long: ", imu_long.shape)
    # print("IMU max: ", np.max(imu_long))
    # print("IMU min: ", np.min(imu_long))
    print("Servos long: ", servos_long.shape)
    print("Truth long: ", truth_all_long.shape)

    print("IMU long 2: ", imu_long_2.shape)
    # print("IMU max 2: ", np.max(imu_long_2))
    # print("IMU min 2: ", np.min(imu_long_2))
    print("Servos long 2: ", servos_long_2.shape)
    print("Truth long 2: ", truth_all_long_2.shape)

    print("IMU long 3: ", imu_long_3.shape)
    # print("IMU max 3: ", np.max(imu_long_3))
    # print("IMU min 3: ", np.min(imu_long_3))
    print("Servos long 3: ", servos_long_3.shape)
    print("Truth long 3: ", truth_all_long_3.shape)

    print("IMU long 4: ", imu_long_4.shape)
    # print("IMU max 4: ", np.max(imu_long_4))
    # print("IMU min 4: ", np.min(imu_long_4))
    print("Servos long 4: ", servos_long_4.shape)
    print("Truth long 4: ", truth_all_long_4.shape)

    # Plot and inspect
    # num_channels = truth_long_4.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     plt.plot(imu[200000:250000, i], label=f'Channel {i+1}')
    # # Add a legend
    # plt.legend()
    # plt.show()

    # Plot and inspect
    # num_channels = servos_long.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     plt.plot(servos_long[0:100000, i], label=f'Channel {i+1}')
    # # Add a legend
    # # plt.ylim(np.min(imu), np.max(imu))
    # plt.autoscale(enable=True, axis="both")
    # plt.legend()
    # plt.show()

    # # Reconstruct the matrices
    # # "data.odometryGroundTruth.Htw.x.x", - 0
    # # "data.odometryGroundTruth.Htw.x.y", - 1
    # # "data.odometryGroundTruth.Htw.x.z", - 2
    # # "data.odometryGroundTruth.Htw.y.x", - 3
    # # "data.odometryGroundTruth.Htw.y.y", - 4
    # # "data.odometryGroundTruth.Htw.y.z", - 5
    # # "data.odometryGroundTruth.Htw.z.x", - 6
    # # "data.odometryGroundTruth.Htw.z.y", - 7
    # # "data.odometryGroundTruth.Htw.z.z", - 8
    # # "data.odometryGroundTruth.Htw.t.x", - 9
    # # "data.odometryGroundTruth.Htw.t.y", - 10
    # # "data.odometryGroundTruth.Htw.t.z" - 11
    # Hwts = []
    # for t in truth_all:
    #     m = np.array[[t[0], t[3], t[6], t[9]],[t[1], t[4], t[7], t[10]],[t[2], t[5], t[8], t[11]],[0.0, 0.0, 0.0, 1.0]]
    #     Hwts.append(m)
    # Htws = [np.linalg.inv(m) for m in Hwts]
    # robot_position_in_world_truth = np.array([h[3,:3] for h in Htws])

    # NOTE: Using only position for first tests to see what happens. May add in heading later.
    # Filter the truth down to just position -  this is the last column of the homogenous transform matrix
    # NOTE: 17/04/2024 - Due to fluctuations in the z component, trying with just x, y
    # NOTE: Remember to reshape if adding or removing features

    truth_long = truth_all_long[:, 9:11]
    print("Truth long shape: ", truth_long.shape)
    truth_long_2 = truth_all_long_2[:, 9:11]
    print("Truth long 2 shape: ", truth_long_2.shape)
    truth_long_3 = truth_all_long_3[:, 9:11]
    print("Truth long 3 shape: ", truth_long_3.shape)
    truth_long_4 = truth_all_long_4[:, 9:11]
    print("Truth long 4 shape: ", truth_long_4.shape)

    # Smooth targets using gaussian filter
    # truth_long_smoothed = gaussian_smooth(truth_long, 50)
    # truth_long_2_smoothed = gaussian_smooth(truth_long_2, 50)
    # truth_long_3_smoothed = gaussian_smooth(truth_long_3, 50)
    # truth_long_4_smoothed = gaussian_smooth(truth_long_4, 50)

    # print("truth 1: ", truth_long.shape)
    # print("Smoothed truth 1: ", truth_long_smoothed.shape)

    # Plot and inspect
    # num_channels = truth_long.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     plt.plot(truth_long[20000:50000, i], label=f'Unsmoothed {i+1}')
    #     plt.plot(truth_long_smoothed[20000:50000, i], label=f'Smoothed {i+1}')
    # # Add a legend
    # # plt.ylim(np.min(imu), np.max(imu))
    # plt.autoscale(enable=True, axis="both")
    # plt.legend()
    # plt.show()

    # join separate arrays
    imu_joined = np.concatenate([imu_long, imu_long_2, imu_long_3, imu_long_4], axis=0)
    servos_joined = np.concatenate([servos_long, servos_long_2, servos_long_3, servos_long_4], axis=0)
    # truth_joined = np.concatenate([truth_long_smoothed, truth_long_2_smoothed, truth_long_3_smoothed, truth_long_4_smoothed], axis=0)
    truth_joined = np.concatenate([truth_long, truth_long_2, truth_long_3, truth_long_4], axis=0)
    # debugs
    print("imu_joined: ", imu_joined.shape)
    print("servos_joined: ",servos_joined.shape)
    print("truth_joined: ",truth_joined.shape)

    # Join the data
    joined_data = np.concatenate([imu_joined, servos_joined, truth_joined], axis=1)
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
    # mean = train_arr.mean(axis=0)
    # std = train_arr.std(axis=0)
    # print("mean: ", mean)
    # print("std: ", std)

    # train_arr = (train_arr - mean) / std
    # validate_arr = (validate_arr - mean) / std
    # test_arr = (test_arr - mean) / std
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
    scaler = MinMaxScaler(feature_range=(-1, 1))
    # Fit scaler to training data only
    scaler.fit(train_arr)
    # Transform the data
    train_arr_scaled = scaler.transform(train_arr)
    validate_arr_scaled = scaler.transform(validate_arr)
    test_arr_scaled = scaler.transform(test_arr)

    # Plot and inspect after normalising
    # num_channels = train_arr.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     plt.plot(train_arr_scaled[900000:935479, i], label=f'Channel {i+1}')
    # # Add a legend
    # plt.legend()
    # plt.show()

    # NOTE: Remember to reshape if adding or removing features

    # Split into data and targets
    # Training
    input_data_train = train_arr_scaled[:, :18]  # imu and servos
    input_targets_train = train_arr_scaled[:, 18:]  # truth
    # Convert targets to relative position
    input_targets_train = convert_to_relative(input_targets_train)

    # Validation
    input_data_validate = validate_arr_scaled[:, :18]  # imu and servos
    input_targets_validate = validate_arr_scaled[:, 18:]  # truth
    # Convert targets to relative position
    input_targets_validate = convert_to_relative(input_targets_validate)

    # Testing
    input_data_test= test_arr_scaled[:, :18]  # imu and servos
    input_targets_test = test_arr_scaled[:, 18:]  # truth
    # Convert targets to relative position
    input_targets_test = convert_to_relative(input_targets_test)

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
    # num_channels = input_targets_train.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     plt.plot(input_targets_train[0:50000, i], label=f'Channel {i+1}')
    # # Add a legend
    # plt.legend()
    # plt.show()

    # NOTE: Samples are roughly 115/sec
    system_sample_rate = 115
    sequence_length = system_sample_rate * 2   # Look back n seconds (system_sample_rate * n). system_sample_rate was roughly calculated at 115/sec
    sequence_stride = 1                         # Shift one sequence_length at a time (rolling window)
    sampling_rate = 1                           # Used for downsampling
    batch_size = 250                          # Number of samples per gradient update (original: 64, seemed better?: 512)

    # Sequence lengths (for return sequences)
    # train_seq_length = input_data_train.shape[0]
    # val_seq_length = input_data_validate.shape[0]
    # test_seq_length = input_data_test.shape[0]

    train_dataset = tf.keras.utils.timeseries_dataset_from_array(
        data=input_data_train,
        targets=input_targets_train,
        sequence_length=sequence_length,
        sequence_stride=sequence_stride,
        sampling_rate=sampling_rate,
        batch_size=batch_size
    )

    validate_dataset = tf.keras.utils.timeseries_dataset_from_array(
        data=input_data_validate,
        targets=input_targets_validate,
        sequence_length=sequence_length,
        sequence_stride=sequence_stride,
        sampling_rate=sampling_rate,
        batch_size=batch_size
    )

    test_dataset = tf.keras.utils.timeseries_dataset_from_array(
        data=input_data_test,
        targets=input_targets_test,
        sequence_length=sequence_length,
        sequence_stride=sequence_stride,
        sampling_rate=sampling_rate,
        batch_size=batch_size
    )

    # Model parameters
    learning_rate = 0.0003   # Controls how much to change the model in response to error.
    epochs = 200             #
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
    # loss_function = keras.losses.log_cosh(y_true=1,y_pred=1)
    # loss_function = keras.losses.Quantile(quantile=0.5)
    # loss_function = quantile_loss????
    # loss_function = keras.losses.Huber(delta=0.5)

    # ** Optimizers **
    # LR schedules
    size_of_dataset = input_data_train.shape[0]
    decay_to_epoch = 20                                         # Number of epochs for learning rate to decay over before it resets
    steps_per_epoch = size_of_dataset // batch_size              # Calculate the number of steps per epoch
    decay_over_steps = decay_to_epoch * steps_per_epoch         # Calculate the number of steps to decay over (scheduler takes the values in steps)
    print(f"Number of steps to decay over before LR resets: {decay_over_steps}")
    lr_schedule = keras.optimizers.schedules.CosineDecayRestarts(initial_learning_rate=learning_rate, first_decay_steps=decay_over_steps, t_mul=1.0, m_mul=1.0, alpha=0.0000005)

    # standard optimisers
    # optimizer = keras.optimizers.Adam(learning_rate=lr_schedule, beta_1=0.90)
    optimizer = keras.optimizers.AdamW(learning_rate=lr_schedule)
    # optimizer = keras.optimizers.AdamW(learning_rate=learning_rate)

    # optimizer=keras.optimizers.Adadelta(learning_rate=lr_schedule)
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

    # Regulariser
    # regularizer1 = keras.regularizers.L1L2(l1=0.00001, l2=0.0001)
    # regularizer2 = keras.regularizers.L1L2(l1=0.00001, l2=0.0001)
    # regularizer3 = keras.regularizers.L1L2(l1=0.00001, l2=0.0001)
    # final_regularizer = keras.regularizers.L1L2(l1=0.002, l2=0.009)

    # Model Layers
    inputs = keras.layers.Input(shape=(sequence_length, input_data_train.shape[1]))
    dropout = keras.layers.Dropout(rate=0.3)(inputs)
    lstm = keras.layers.LSTM(10, kernel_initializer=keras.initializers.HeNormal(), kernel_regularizer=keras.regularizers.L1L2(l1=0.000035, l2=0.00035), return_sequences=True)(dropout)    # 32 originally
    normalise = keras.layers.LayerNormalization()(lstm)

    # lstm2 = keras.layers.LSTM(30, kernel_initializer=keras.initializers.HeNormal(), kernel_regularizer=keras.regularizers.L1L2(l1=0.000035, l2=0.00035), return_sequences=True)(normalise)    # 32 originally
    # normalise2 = keras.layers.LayerNormalization()(lstm2)

    # lstm3 = keras.layers.LSTM(30, kernel_initializer=keras.initializers.HeNormal(), kernel_regularizer=keras.regularizers.L1L2(l1=0.000035, l2=0.00035), return_sequences=True)(normalise2)    # 32 originally
    # normalise3 = keras.layers.LayerNormalization()(lstm3)

    # Apply attention layer that considers all normalised lstm outputs
    # attention = keras.layers.Attention()([normalise, normalise])
    # Compute dot product between attention weights and last LSTM layer
    # context_vector = keras.layers.Dot(axes=(1, 1))([attention, normalise])

    # lstm4 = keras.layers.LSTM(30, kernel_initializer=keras.initializers.HeNormal(), kernel_regularizer=keras.regularizers.L1L2(l1=0.00003, l2=0.0003), return_sequences=False)(dropout3)    # 32 originally
    # normalise4 = keras.layers.LayerNormalization()(lstm4)
    # dropout4 = keras.layers.Dropout(rate=0.35)(normalise4)
    # NOTE: Changed dense layer units to 2 due to removing z component
    # dense1 = keras.layers.Dense(16, kernel_regularizer=keras.regularizers.L1L2(l1=0.00001, l2=0.0002))(dropout4)
    dense2 = keras.layers.TimeDistributed(keras.layers.Dense(2, kernel_regularizer=keras.regularizers.L1L2(l1=0.00001, l2=0.0002)))(normalise)   # Target shape[1] is 3
    model = keras.Model(inputs=inputs, outputs=dense2)
    model.compile(optimizer=optimizer, loss=loss_function, metrics=["mae"])
    model.summary()

    # Examples
    # lstm = keras.layers.Bidirectional(LSTM(200, return_sequences=True, recurrent_regularizer=keras.regularizers.L1L2(l1=0.0002, l2=0.006)))(dropout)
    # lstm2 = keras.layers.LSTM(50, return_sequences=True, kernel_initializer=keras.initializers.HeUniform(), kernel_regularizer=keras.regularizers.L1L2(l1=0.00001, l2=0.0002), recurrent_regularizer=keras.regularizers.L1L2(l1=0.00001, l2=0.0002))(dropout)    # 32 originally
    # dropout2 = keras.layers.Dropout(rate=0.2)(lstm2)

    # lstm3 = keras.layers.LSTM(10, return_sequences=False, kernel_initializer=keras.initializers.HeUniform(), kernel_regularizer=keras.regularizers.L1L2(l1=0.00001, l2=0.0002), recurrent_regularizer=keras.regularizers.L1L2(l1=0.00001, l2=0.0002))(dropout2)    # 32 originally
    # dropout3 = keras.layers.Dropout(rate=0.2)(lstm3)

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
