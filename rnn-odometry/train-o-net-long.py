#!/usr/bin/env python3

import argparse
import os
import subprocess

import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
from sklearn.preprocessing import MinMaxScaler


def main():

    # numpy arrays
    imu = np.load("processed-outputs/numpy/long/1/long-imu-1.npy")
    servos = np.load("processed-outputs/numpy/long/1/long-servos-1.npy")
    truth = np.load("processed-outputs/numpy/long/1/long-truth-1.npy")
    # tstamps = np.load('processed-outputs/numpy/long/1/long-tstamps-1.npy')

    # Join the data
    joined_data = np.concatenate([imu, servos, truth], axis=1)
    print(joined_data.shape)
    training_size = 0.5
    validate_size = 0.2

    # Split the data into training, validation and testing sets
    num_time_steps = joined_data.shape[0]
    num_train = int(num_time_steps * training_size)
    num_val = int(num_time_steps * validate_size)

    train_arr = joined_data[:num_train]

    mean = train_arr.mean(axis=0)
    std = train_arr.std(axis=0)

    train_arr = (train_arr - mean) / std
    validate_arr = (joined_data[num_train : (num_train + num_val)] - mean) / std
    test_arr = (joined_data[(num_train + num_val) :] - mean) / std

    print(f"Training set size: {train_arr.shape}")
    print(f"Validation set size: {validate_arr.shape}")
    print(f"Test set size: {test_arr.shape}")


if __name__ == "__main__":
    main()
