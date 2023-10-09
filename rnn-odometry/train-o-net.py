#!/usr/bin/env python3

import argparse
import json
import os
import subprocess

import numpy as np


def main():
    parser = argparse.ArgumentParser(description="Train an LSTM-RNN on servo and imu data with XYZ ground truth.")
    parser.add_argument(
        "path_to_data",
        help="The path to the directory holding the training data." + "The program will search nested directories.",
    )

    args = parser.parse_args()
    path_to_data = args.path_to_data

    # Declare both features and ground truth arrays
    servo_data = []
    imu_data = []
    truth = []
    features = [[], []]
    samples = 0

    # NOTE: Should be

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
                servo_data.append(np.load(filepath))
            if "imu" in file:
                print("imu: " + file)
                imu_data.append(np.load(filepath))
            if "truth" in file:
                print("truth: " + file)
                truth.append(np.load(filepath))
            # features[0].append([servo_data])
            # features[1].append([imu_data])
    # NP Concatenate or Stack
    print("Servos list: ")
    print(servo_data)
    print("IMU list: ")
    print(imu_data)
    print("Ground truth list: ")
    print(samples)
    print(truth)


if __name__ == "__main__":
    main()
