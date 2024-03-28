#!/usr/bin/env python3

import argparse
import datetime
import os
import subprocess

import keras
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
from sklearn.preprocessing import MinMaxScaler


def main():

    # numpy arrays
    imu = np.load("processed-outputs/numpy/long/1/long-imu-1.npy")
    servos = np.load("processed-outputs/numpy/long/1/long-servos-1.npy")
    truth_all = np.load("processed-outputs/numpy/long/1/long-truth-1.npy")
    # tstamps = np.load('processed-outputs/numpy/long/1/long-tstamps-1.npy')
    print(imu.shape)
    print("IMU max", np.max(imu))
    print("IMU min",np.min(imu))
    print(servos.shape)
    print(truth_all.shape)

    # Plot and inspect
    # num_channels = imu.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     plt.plot(imu[200000:250000, i], label=f'Channel {i+1}')
    # # Add a legend
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
    truth = truth_all[:, 9:12]
    print(truth.shape)
    # "data.accelerometer.x",
    # "data.accelerometer.y",
    # "data.accelerometer.z",
    # "data.gyroscope.x",
    # "data.gyroscope.y",
    # "data.gyroscope.z"

    # Add buffers to the imu data (maybe not needed?)

    # imu_buffers = [10.0, 10.0, 10.0, 50.0, 50.0, 50.0]
    # imu_maxes = np.max(imu, axis = 0) + imu_buffers
    # imu_mins = np.min(imu, axis = 0) - imu_buffers
    # # Cap the imu data off to remove outliers
    # imu_capped = np.maximum(np.minimum(imu, imu_maxes, axis = 1), imu_mins, axis = 1)
    # normalized_imu  = ((imu_capped + imu_mins) / (imu_maxes + imu_mins)) - 0.5
    # plot this
    # import pdb
    # pdb.set_trace()

    # Clip the IMU data to reduce spikes - Putting this after normalisation
    # imu_clipped = np.clip(imu, -10, 10)

    # Plot and inspect
    # num_channels = imu.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     plt.plot(imu[0:50000, i], label=f'Channel {i+1}')
    # # Add a legend
    # # plt.ylim(np.min(imu), np.max(imu))
    # plt.autoscale(enable=True, axis="both")
    # plt.legend()
    # plt.show()


    # Join the data
    joined_data = np.concatenate([imu, servos, truth], axis=1)
    print(joined_data.shape)

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

    # Normalise -
    # NOTE: mean and std from training dataset is used to normalise
    # all of the datasets to prevent information leakage.
    # mean and std from the training run will need to be used in production for de-normalising the predictions.
    mean = train_arr.mean(axis=0)
    std = train_arr.std(axis=0)
    print("mean: ", mean)
    print("std: ", std)

    train_arr = (train_arr - mean) / std
    validate_arr = (validate_arr - mean) / std
    test_arr = (test_arr - mean) / std

    print(f"Training set size: {train_arr.shape}")
    print("Training set min:", np.min(train_arr))
    print("Training set max:", np.max(train_arr))
    print(f"Validation set size: {validate_arr.shape}")
    print(f"Test set size: {test_arr.shape}")

    # clip the outliers in the data
    train_arr_clipped = np.clip(train_arr, -10, 10)
    validate_arr_clipped = np.clip(validate_arr, -10, 10)
    test_arr_clipped = np.clip(test_arr, -10, 10)

    # Plot and inspect after normalising
    # num_channels = train_arr.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     plt.plot(train_arr_clipped[100000:200000, i], label=f'Channel {i+1}')
    # # Add a legend
    # plt.legend()
    # plt.show()

    # Split into data and targets
    # Training
    input_data_train = train_arr_clipped[:, :26]  # imu and servos
    input_targets_train = train_arr_clipped[:, 26:]  # truth
    # Validation
    input_data_validate = validate_arr_clipped[:, :26]  # imu and servos
    input_targets_validate = validate_arr_clipped[:, 26:]  # truth
    # Testing
    input_data_test= test_arr_clipped[:, :26]  # imu and servos
    input_targets_test = test_arr_clipped[:, 26:]  # truth

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
    # num_channels = input_data_train.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(6, 26):
    #     plt.plot(input_data_train[100000:202280, i], label=f'Channel {i+1}')
    # # Add a legend
    # plt.legend()
    # plt.show()

    # NOTE: Samples are roughly 115/sec
    system_sample_rate = 115
    sequence_length = system_sample_rate * 4   # Look back n seconds (system_sample_rate * n). system_sample_rate was roughly calculated at 115/sec
    sequence_stride = 1                         # Shift one sequence_length at a time (rolling window)
    sampling_rate = 1                           # Used for downsampling
    batch_size = 1000                           # Number of samples per gradient update (original: 64, seemed better?: 512)

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
    epochs = 150             #

    # Loss functions
    loss_function = keras.losses.MeanAbsoluteError()
    # loss_function = keras.losses.log_cosh(y_true=1,y_pred=1)
    # loss_function = keras.losses.Quantile(quantile=0.5)
    # loss_function = quantile_loss????
    # loss_function = keras.losses.Huber(delta=0.5)

    # Optimizers
    optimizer=keras.optimizers.Adam(learning_rate=learning_rate)
    # optimizer=keras.optimizers.Adadelta(learning_rate=learning_rate)

    # Tensorboard
    timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    log_dir = "logs/fit/" + timestamp
    tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir=log_dir, histogram_freq=1)

    # Regulariser
    # regulariser = keras.regularizers.L1L2(0.01)

    # Model Layers
    inputs = keras.layers.Input(shape=(sequence_length, input_data_train.shape[1]))

    lstm_out = keras.layers.LSTM(64, return_sequences=True)(inputs)    # 32 originally

    lstm_out2 = keras.layers.LSTM(26, return_sequences=False)(lstm_out)

    outputs = keras.layers.Dense(3)(lstm_out2)   # Target shape[1] is 3
    model = keras.Model(inputs=inputs, outputs=outputs)
    model.compile(optimizer=optimizer, loss=loss_function)
    model.summary()

    model.fit(
        train_dataset,
        validation_data=validate_dataset,
        epochs=epochs,
        callbacks=[tensorboard_callback]
    )

    # Note add back the model save
    model.save("models/model-" + timestamp)


if __name__ == "__main__":
    main()
