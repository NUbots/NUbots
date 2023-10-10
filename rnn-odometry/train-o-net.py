#!/usr/bin/env python3

import argparse
import os
import subprocess

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
    parser = argparse.ArgumentParser(description="Train an LSTM-RNN on servo and imu data with XYZ ground truth.")
    parser.add_argument(
        "path_to_data",
        help="The path to the directory holding the training data." + "The program will search nested directories.",
    )

    args = parser.parse_args()
    path_to_data = args.path_to_data

    # Declare both features and ground truth lists
    servo_data_list = []
    imu_data_list = []
    truth_list = []
    features = [[], []]
    samples = 0

    # Walk the given file path searching for data.
    for root, dirs, files in os.walk(path_to_data):
        for file in sorted(files):
            # increment sample count
            samples = samples + 1
            # Concatenate the imu and servo data along the second axis
            # This is necessary to maintain their temporal relationship
            filepath = os.path.join(root, file)
            if "servos" in file:
                print("Servos: " + file)
                servo_data_list.append(np.load(filepath))
            if "imu" in file:
                print("imu: " + file)
                imu_data_list.append(np.load(filepath))
            if "truth" in file:
                print("truth: " + file)
                truth_list.append(np.load(filepath))
            # features[0].append([servo_data_list])
            # features[1].append([imu_data_list])
    # NP Concatenate or Stack?

    # Create np arrays
    # Servos
    for item in servo_data_list:
        print(item)
        servo_data_arr = np.concatenate((item), axis=0)
    # # Imu
    # for item in imu_data_list:
    #     imu_data_arr = np.concatenate(item)
    # # Truth
    # for item in truth_list:
    #     truth_arr = np.concatenate(item)

    # print("Servos list: ")
    # print(servo_data_list)
    # print("IMU list: ")
    # print(imu_data_list)
    # print("Ground truth list: ")
    # print(samples)
    # print(truth_list)
    # print("SHAPES/LENGTHS")
    print("servos")
    print(servo_data_arr)
    print(servo_data_arr.shape)
    # print("imu")
    # print(imu_data_arr)
    # print(imu_data_arr.shape)
    # print("imu")
    # print(truth_arr)
    # print(truth_arr.shape)
    # normalise
    # scaler = MinMaxScaler()
    # normalised_servo_tensor = scaler.fit_transform(servo_data_list)
    # normalised_imu_tensor = scaler.fit_transform(imu_data_list)


if __name__ == "__main__":
    main()
