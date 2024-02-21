#!/usr/bin/env python3

import argparse
import os
import subprocess

import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
from sklearn.preprocessing import MinMaxScaler

# def normalise(list):
#     # convert list to np
#     arr = np.array(list)
#     # Convert the arrays to tensors
#     # tensors = [tf.convert_to_tensor(array, dtype=tf.float32) for array in arrays]
#     tensors = tf.convert_to_tensor(arr, dtype=tf.float32)

#     # Concatenate all tensors into one
#     concatenated = tf.concat(tensors, axis=0)

#     # Calculate the min and max of the concatenated tensor
#     min_val = tf.reduce_min(concatenated)
#     max_val = tf.reduce_max(concatenated)

#     # Normalize each tensor
#     normalised_tensors = [(tensor - min_val) / (max_val - min_val) for tensor in tensors]
#     return normalised_tensors


def main():
    # parser = argparse.ArgumentParser(description="Train an LSTM-RNN on servo and imu data with XYZ ground truth.")
    # parser.add_argument(
    #     "path_to_data",
    #     help="The path to the directory holding the training data." + "The program will search nested directories.",
    # )

    # args = parser.parse_args()
    # path_to_data = args.path_to_data

    # numpy arrays
    imu = np.load('processed-outputs/numpy/long/1/long-imu-1.npy')
    servos = np.load('processed-outputs/numpy/long/1/long-servos-1.npy')
    truth = np.load('processed-outputs/numpy/long/1/long-truth-1.npy')
    tstamps = np.load('processed-outputs/numpy/long/1/long-tstamps-1.npy')

    # Sizes
    print('Size of imu data: ' + str(imu.shape))
    print('Size of servo data: ' + str(servos.shape))
    print('Size of truth data: ' + str(truth.shape))
    print('Size of tstamps data: ' + str(tstamps.shape))

    plt.close('all')

    plot_num = 1000

    # plot
    fig, ax = plt.subplots(2, 1, figsize=(18, 6), sharex = True)
    ax[0].plot(imu[:plot_num, [0, 1]], 'o-')
    ax[0].plot(imu[:plot_num, [2]]-np.array([9.8]), 'o-')
    ax[1].plot(imu[:plot_num, [3, 4, 5]], 'o-')
    ax[0].legend([
        "accelerometer.x",
        "accelerometer.y",
        "accelerometer.z",
    ])
    ax[1].legend([
        "gyroscope.x",
        "gyroscope.y",
        "gyroscope.z"
    ])

    # plt.figure()
    # tstamps_diff = (tstamps[1:] - tstamps[:-1])
    # plt.plot(tstamps_diff[:plot_num], 'o-')


    plt.show()



if __name__ == "__main__":
    main()
