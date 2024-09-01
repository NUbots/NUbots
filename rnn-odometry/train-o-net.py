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

    def partition_dataset(features, targets, window_size):
        inputs = []
        outputs = []

        for i in range(0, len(features) - window_size):
            window = features[i:i + window_size]
            next_value = targets[i + window_size]

            inputs.append(window)
            outputs.append(next_value)

        return np.array(inputs), np.array(outputs)

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
    first_file = 1
    num_files = 20  # Number of files to load
    skip_files = []  # Files to skip - start stop runs: 8,14,15,22,25
    prefix = "s-new"  # s for straight path
    imu = []
    servos = []
    truth_all = []
    truth_start_end_indicator = []

    for i in range(first_file, num_files + 1):

        # Check that the start and end files are not in the skip_files list
        if first_file in skip_files or num_files in skip_files:
            print("Error: Start or end file in skip_files list.")
            # Exit program
            break

        # Skip files in the skip_files list
        if i in skip_files:
            print(f"Skipping file {i}")
            continue

        # Load the data
        imu_data = np.load(f"processed-outputs/numpy/{prefix}/{i}/{prefix}-imu-{i}.npy")
        imu.append(imu_data)
        servos_data = np.load(f"processed-outputs/numpy/{prefix}/{i}/{prefix}-servos-{i}.npy")
        servos.append(servos_data)
        truth_data = np.load(f"processed-outputs/numpy/{prefix}/{i}/{prefix}-truth-{i}.npy")
        # Convert the loaded chunk of truth data to relative positions
        truth_data = convert_to_relative(truth_data)
        truth_all.append(truth_data)

        # # Plot the data as it's loaded
        # plt.figure(figsize=(12, 8))

        # # Plot IMU data
        # plt.subplot(3, 1, 1)
        # plt.plot(imu_data)
        # plt.title(f'IMU Data - File {i}')
        # plt.xlabel('Sample')
        # plt.ylabel('IMU Values')

        # # Plot Servos data
        # # plt.subplot(3, 1, 2)
        # # plt.plot(servos_data)
        # # plt.title(f'Servos Data - File {i}')
        # # plt.xlabel('Sample')
        # # plt.ylabel('Servos Values')

        # # Plot Truth data
        # plt.subplot(3, 1, 3)
        # plt.plot(truth_data)
        # plt.title(f'Truth Data - File {i}')
        # plt.xlabel('Sample')
        # plt.ylabel('Truth Values')

        # # Show the plot
        # plt.tight_layout()
        # plt.show()

        # pause to inspect each plot
        # input("Press Enter to continue to the next file...")
        # plt.close()

        # Create a chunk for the truth_start_end_indicator array
        chunk_size = truth_data.shape[0]  # Assuming the first dimension is the one we're interested in
        indicator_chunk = np.zeros(chunk_size)
        indicator_chunk[0] = 1  # Start indicator
        indicator_chunk[-1] = 1  # End indicator
        truth_start_end_indicator.extend(indicator_chunk)

    # Need to do the relative conversions here
    # Use the convert_to_relative function to convert the truth data to relative positions
    # NOTE: Htw is already relative to the starting point!! But the starting point can vary if not converted
    # truth_all = [convert_to_relative(truth) for truth in truth_all]

    # Smoothing should be done here
    # Loop through each truth array and smooth
    # truth_all = [gaussian_smooth(truth, 50) for truth in truth_all]

    # Loop through each array and concatenate into a numpy array
    imu_joined = np.concatenate(imu, axis=0)
    servos_joined = np.concatenate(servos, axis=0)
    truth_all_joined = np.concatenate(truth_all, axis=0)
    truth_start_end_indicator_joined = np.array(truth_start_end_indicator)

    # Print the shape of the joined arrays
    print("IMU s joined: ", imu_joined.shape)
    print("Servos s joined: ", servos_joined.shape)
    print("Truth s joined: ", truth_all_joined.shape)
    print("Truth start/end indicator: ", truth_start_end_indicator_joined.shape)

    # Slice out the arm and head servos
    servos_joined = servos_joined[:, 6:18]
    # Print the shape of the sliced servos
    print("Servos s sliced: ", servos_joined.shape)

    # Plot and inspect each joined array
    # num_channels = servos_joined.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     plt.plot(servos_joined[:, i], label=f'imu {i+1}')
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
    # num_channels = servos_joined.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     plt.plot(servos_joined[100000:, i], label=f'Servos {i+1}')
    # # Add a legend
    # plt.legend()
    # plt.show()


    # Join the data
    joined_data = np.concatenate([imu_joined, servos_joined, truth_joined_sliced], axis=1)

    # Testing without the servos
    # joined_data = np.concatenate([imu_joined, truth_joined_sliced], axis=1)

    print("Total joined data shape: ", joined_data.shape)

    # Plot and inspect after joining
    # num_channels = joined_data.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     plt.plot(joined_data[:, i], label=f'Channel {i+1}')
    # # Add a legend
    # plt.legend()
    # plt.show()


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

    # Plot and inspect after splitting
    # num_channels = train_arr.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(6):
    #     plt.plot(train_arr[:, i], label=f'Channel {i+1}')
    # # Add a legend
    # plt.legend()
    # plt.show()

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

    # # clip the outliers in the data
    # train_arr = np.clip(train_arr, -24, 24)
    # validate_arr = np.clip(validate_arr, -24, 24)
    # test_arr = np.clip(test_arr, -24, 24)

    ## End of standardisation ##

    ## Min/Max Scaling ##

    scaler = MinMaxScaler(feature_range=(-1, 1))
    # Fit scaler to training data only
    scaler.fit(train_arr)
    # Transform the data
    train_arr = scaler.transform(train_arr)
    validate_arr = scaler.transform(validate_arr)
    test_arr = scaler.transform(test_arr)

    ## End of Min/Max Scaling ##

    # Print the shapes and min/max values of the datasets
    print(f"Training set size: {train_arr.shape}")
    print("Training set min:", np.min(train_arr))
    print("Training set max:", np.max(train_arr))
    print(f"Validation set size: {validate_arr.shape}")
    print(f"Test set size: {test_arr.shape}")

    # Plot and inspect after normalising
    # num_channels = train_arr.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(6):
    #     plt.plot(train_arr[:, i], label=f'Channel {i+1}')
    # # Add a legend
    # plt.legend()
    # plt.show()

    #### End of normalisation ####



    #### Split into data and targets ####
    # NOTE: Remember to reshape if adding or removing features
    # NOTE: Just using IMU data, so changing index slice from 18 to 6
    # Training
    # Use when using the servos
    input_data_train = train_arr[:, :18]  # imu and servos
    input_targets_train = train_arr[:, 18:]  # truth
    # Use when not using the servos
    # input_data_train = train_arr[:, :6]  # imu and servos
    # input_targets_train = train_arr[:, 6:]  # truth

    # Convert sliced targets to relative position
    # input_targets_train = convert_to_relative(input_targets_train)

    # Validation
    # Use when using the servos
    input_data_validate = validate_arr[:, :18]  # imu and servos
    input_targets_validate = validate_arr[:, 18:]  # truth
    # Use when not using the servos
    # input_data_validate = validate_arr[:, :6]  # imu and servos
    # input_targets_validate = validate_arr[:, 6:]  # truth

    # Convert sliced targets to relative position
    # input_targets_validate = convert_to_relative(input_targets_validate)

    # Testing
    # Use when using the servos
    input_data_test= test_arr[:, :18]  # imu and servos
    input_targets_test = test_arr[:, 18:]  # truth
    # Use when not using the servos
    # input_data_test= test_arr[:, :6]  # imu and servos
    # input_targets_test = test_arr[:, 6:]  # truth

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

    #### End of splitting ####

    #### Add indicator to the datasets ####
    # Convert indicator list to a numpy array before reshaping
    truth_start_end_indicator_2d = np.array(truth_start_end_indicator).reshape(-1, 1)

    # Split the indicator array
    train_indicator = truth_start_end_indicator_2d[:train_arr.shape[0]]
    validate_indicator = truth_start_end_indicator_2d[train_arr.shape[0]:train_arr.shape[0] + validate_arr.shape[0]]
    test_indicator = truth_start_end_indicator_2d[train_arr.shape[0] + validate_arr.shape[0]:]
    # Concatenate the indicator arrays to the main datasets
    input_data_train = np.concatenate((input_data_train, train_indicator.reshape(-1, 1)), axis=1)
    input_data_validate = np.concatenate((input_data_validate, validate_indicator.reshape(-1, 1)), axis=1)
    input_data_test = np.concatenate((input_data_test, test_indicator.reshape(-1, 1)), axis=1)
    #### End of adding indicator ####

    # print dataset shapes (will have an extra column for the indicator)
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

    # Plot and inspect after adding indicator
    # num_channels = input_data_train.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     plt.plot(input_data_train[:, i], label=f'Data {i+1}')
    # # Add a legend
    # plt.legend()
    # plt.show()

    # Plot and inspect (use only if training with servos)
    # num_channels = input_data_test.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     if i < 6:
    #         plt.subplot(4, 1, 1)
    #         plt.plot(input_data_test[0:50000, i], label=f'Imu {i+1}')
    #         plt.legend()
    #     elif i < 18:
    #         plt.subplot(4, 1, 2)
    #         plt.plot(input_data_test[0:50000, i], label=f'Servos {i+1}')
    #         plt.legend()
    #     else:
    #         plt.subplot(4, 1, 3)
    #         plt.plot(input_data_test[0:50000, i], label=f'Indicator {i+1}')
    #         plt.legend()
    # # Also plot the truth on another subplot
    # plt.subplot(4, 1, 4)
    # plt.plot(input_targets_test[0:50000, 0], label=f'Target 1')
    # plt.plot(input_targets_test[0:50000, 1], label=f'Target 2')
    # plt.legend()
    # plt.show()

    # Plot and inspect (use only if training without servos)
    # num_channels = input_data_train.shape[1]
    # plt.figure(figsize=(10, 5))
    # # Plot each channel
    # for i in range(num_channels):
    #     if i < 6:
    #         plt.subplot(3, 1, 1)
    #         plt.plot(input_data_train[:, i], label=f'Imu {i+1}')
    #         plt.legend()
    #     else:
    #         plt.subplot(3, 1, 2)
    #         plt.plot(input_data_train[:, i], label=f'Indicator {i+1}')
    #         plt.legend()
    # # Also plot the truth on another subplot
    # plt.subplot(3, 1, 3)
    # plt.plot(input_targets_train[:, 0], label=f'Target 1')
    # plt.plot(input_targets_train[:, 1], label=f'Target 2')
    # plt.legend()
    # plt.show()

    # NOTE: Samples are roughly 115/sec
    # system_sample_rate = 115
    sequence_length = 25   # Look back n seconds (system_sample_rate * n). system_sample_rate was roughly calculated at 115/sec
    # sequence_stride = 1                         # Shift one sequence_length at a time (rolling window)
    # sampling_rate = 1                           # Used for downsampling
    batch_size = 256                         # Number of samples per gradient update (original: 64, seemed better?: 512)

    # Partition the training and validation datasets into sequences
    input_data_train, input_targets_train = partition_dataset(input_data_train, input_targets_train, sequence_length)
    input_data_validate, input_targets_validate = partition_dataset(input_data_validate, input_targets_validate, sequence_length)
    validation_data = (input_data_validate, input_targets_validate)
    # Test for any bugs by using the training data as validation data
    # validation_data = (input_data_train, input_targets_train)

    # Print the shapes of the partitioned datasets
    print(f"input_data_train: {input_data_train.shape}")
    print(f"input_targets_train: {input_targets_train.shape}")
    print(f"input_data_validate: {input_data_validate.shape}")
    # Print the shape of the second element in the training dataset

    # Model parameters
    learning_rate = 0.0001   # Controls how much to change the model in response to error.
    epochs = 500
    # loss_function = keras.losses.MeanSquaredError()
    loss_function = keras.losses.MeanAbsoluteError()
    # loss_function = keras.losses.Huber()

    # Random seed
    tf.random.set_seed(65)

    # ** Optimisers **
    # LR schedules
    # size_of_dataset = input_data_train.shape[0]
    # decay_to_epoch = 50                                         # Number of epochs for learning rate to decay over before it resets
    # steps_per_epoch = size_of_dataset // batch_size             # Calculate the number of steps per epoch
    # decay_over_steps = decay_to_epoch * steps_per_epoch         # Calculate the number of steps to decay over (scheduler takes the values in steps)
    # print(f"Decay to epoch: {decay_to_epoch}")
    # print(f"Number of steps to decay over before LR resets: {decay_over_steps}")

    # LR schedule to decay the learning rate over a given number of steps (epochs calculated above), then it will reset.
    # lr_schedule = keras.optimizers.schedules.CosineDecayRestarts(initial_learning_rate=learning_rate, first_decay_steps=decay_over_steps, t_mul=1.00, m_mul=1.08, alpha=0.00001)

    # LR schedule to decrease learning rate by 0.1 after 100 epochs
    # lr_schedule = keras.optimizers.schedules.ExponentialDecay(initial_learning_rate=learning_rate, decay_steps=decay_over_steps, decay_rate=0.01, staircase=True)


    # Static learning rate
    # optimizer = keras.optimizers.Adam(learning_rate=learning_rate)
    optimizer = keras.optimizers.AdamW(learning_rate=learning_rate)
    # optimizer=keras.optimizers.Adadelta(learning_rate=learning_rate)
    # optimizer = keras.optimizers.Adamax(learning_rate=learning_rate)

    # Used with the LR schedule
    # optimizer = keras.optimizers.AdamW(learning_rate=lr_schedule)
    # optimizer=keras.optimizers.Adadelta(learning_rate=lr_schedule)

    # Model Layers
    inputs = keras.layers.Input(shape=(sequence_length, input_data_train.shape[2]))

    lstm = keras.layers.LSTM(16, kernel_initializer=keras.initializers.GlorotNormal(), return_sequences=False)(inputs)
    batch_norm = keras.layers.BatchNormalization()(lstm)
    dropout = keras.layers.Dropout(rate=0.25)(batch_norm)

    # lstm2 = keras.layers.LSTM(2, kernel_initializer=keras.initializers.GlorotNormal(), kernel_regularizer=keras.regularizers.L2(0.04), return_sequences=False)(batch_norm)    # 32 originally
    # batch_norm2 = keras.layers.BatchNormalization()(lstm2)
    # dropout2 = keras.layers.Dropout(rate=0.25)(batch_norm2)

    # normalise = keras.layers.LayerNormalization()(batch_norm)

    # normalise = keras.layers.LayerNormalization()(normalise)
    outputs = keras.layers.Dense(2)(dropout)

    model = keras.Model(inputs=inputs, outputs=outputs)
    model.compile(optimizer=optimizer, loss=loss_function)
    model.summary()

    # Callbacks
    # early_stopping_callback = keras.callbacks.EarlyStopping(monitor='val_loss', patience=20, restore_best_weights=True)
    # checkpoint_callback = keras.callbacks.ModelCheckpoint(filepath='best_model.h5', monitor='val_loss', save_best_only=True, mode='min', verbose=1)
    # reduce_lr_callback = keras.callbacks.ReduceLROnPlateau(monitor='val_loss', factor=0.5, patience=5, min_lr=1e-6, verbose=1)
    # Tensorboard
    timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    log_dir = "logs/fit/" + timestamp
    tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir=log_dir, histogram_freq=1)

    # Train the model
    model.fit(
        x=input_data_train,
        y=input_targets_train,
        validation_data=validation_data,
        epochs=epochs,
        callbacks=[tensorboard_callback],
        # Unspecified batch size will default to 32
        batch_size=batch_size
    )

    # Note add back the model save
    model.save("models/model-" + timestamp)



if __name__ == "__main__":
    main()
