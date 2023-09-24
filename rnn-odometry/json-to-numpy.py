#!/usr/bin/env python3

import json
import sys

import numpy as np


def convert_json_to_numpy(input_file, output_file):
    with open(input_file) as f:
        data = json.load(f)

    values = []
    for item in data:
        values.append(list(item.values()))

    arr = np.array(values)
    print("Printing numpy array")
    print("Array size: " + str(arr.size))
    print(arr)
    # Save the numpy array to a file
    np.save(output_file, arr)
    # np.savetxt(output_file, arr, delimiter=",")


if __name__ == "__main__":
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    convert_json_to_numpy(input_file, output_file)
