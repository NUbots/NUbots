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
    #     help="The path to the directory holding the training " + "The program will search nested directories.",
    # )

    # args = parser.parse_args()
    # path_to_data = args.path_to_data

    # numpy arrays
    # imu = np.load('processed-outputs/numpy/s/18/s-imu-18.npy')
    # servos = np.load('processed-outputs/numpy/s/18/s-servos-18.npy')
    truth = np.load('processed-outputs/numpy/s/23/s-truth-23.npy')
    # tstamps = np.load('processed-outputs/numpy/s/18/s-tstamps-18.npy')
    truth2 = np.load('processed-outputs/numpy/s/24/s-truth-24.npy')
    truth3 = np.load('processed-outputs/numpy/s/25/s-truth-25.npy')

    # print the shape of "truth" array
    print(truth.shape)

   # Plot just the first 1000 samples of the truth array
    plt.plot(truth[:10000, 9], label='X: +y > -y')
    plt.plot(truth[:10000, 10], label='Y: +y > -y')
    # plt.plot(truth[:10000, 11], label='Z: +y > -y')
    plt.plot(truth3[:10000, 9], label='X2: -y > +y')
    plt.plot(truth3[:10000, 10], label='Y2: -y > +y')
    # plt.plot(truth3[:10000, 11], label='Z2: -y > +y')
    plt.plot(truth2[:10000, 9], label='X3: +x > -x')
    plt.plot(truth2[:10000, 10], label='Y3: +x > -x')
    # plt.plot(truth2[:10000, 11], label='Z3: +x > -x')
    plt.legend()
    plt.show()

    # # Plot the first 100 samples of the imu array
    # plt.plot(imu[:100, 0], label='X')
    # plt.plot(imu[:100, 1], label='Y')
    # plt.plot(imu[:100, 2], label='Z')
    # plt.legend()
    # plt.show()

    # # Plot the first 100 samples of the servos array
    # plt.plot(servos[:100, 0], label='X')
    # plt.plot(servos[:100, 1], label='Y')
    # plt.plot(servos[:100, 2], label='Z')
    # plt.legend()
    # plt.show()

    # # Plot the first 100 samples of the tstamps array
    # plt.plot(tstamps[:100])
    # plt.show()

    # # Plot the first 100 samples of the imu array
    # plt.plot(imu[:100, 0], label='X')
    # plt.plot(imu[:100, 1], label='Y')
    # plt.plot(imu[:100, 2], label='Z')
    # plt.legend()
    # plt.show()

    # # Plot the first 100 samples of the servos array
    # plt.plot(servos[:100, 0], label='X')
    # plt.plot(servos[:100, 1], label='Y')
    # plt.plot(servos[:100, 2], label='Z')
    # plt.legend()
    # plt.show()

    # # Plot the first 100 samples of the tstamps array
    # plt.plot(tstamps[:100])
    # plt.show()

    # # Plot the first 100 samples of the imu array
    # plt.plot(imu[:100, 0], label='X')
    # plt.plot(imu[:100, 1], label='Y')
    # plt.plot(imu[:100, 2], label='Z')
    # plt.legend()
    # plt.show()

    # # Plot the first 100 samples of the servos array
    # plt.plot(servos[:100, 0], label='X')
    # plt.plot(servos[:100, 1], label='Y')
    # plt.plot(servos[:100, 2], label='Z')



if __name__ == "__main__":
    main()
