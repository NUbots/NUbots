#!/usr/bin/env python3

import argparse
import json
import os
import subprocess


def main():
    parser = argparse.ArgumentParser(description="Process nuclear output json files into pieces for training")
    parser.add_argument("config_dir", help="Path to the directory holding the config files")
    parser.add_argument("input_file", help="Path to the nuclear input file")
    parser.add_argument("json_output_path", help="Path to the output directory to hold the processed json data")
    parser.add_argument("npy_output_path", help="Path to the output directory to hold the processed numpy data")
    parser.add_argument(
        "file_prefix", help="The prefix to add for identifying the type of path represented by the data"
    )

    args = parser.parse_args()
    config_dir = args.config_dir
    input_file = args.input_file
    file_prefix = args.file_prefix
    json_output_path = args.json_output_path
    npy_output_path = args.npy_output_path
    print(os.getcwd())

    # get the contents of the config directory

    config_dir_contents = os.listdir(config_dir)
    # Create the directory if it doesn't exist
    os.makedirs(json_output_path + "/" + file_prefix, exist_ok=True)
    # count the size of the json output directory
    json_output_dir_contents = os.listdir(json_output_path + "/" + file_prefix)
    json_output_dir_count = 0
    for item in json_output_dir_contents:
        json_output_dir_count += 1
    print("Json output dir count: " + str(json_output_dir_count))
    # increment
    json_output_dir_count += 1

    # Create the directory if it doesn't exist
    os.makedirs(npy_output_path + "/" + file_prefix, exist_ok=True)
    # count the size of the npy output directory
    npy_output_dir_contents = os.listdir(npy_output_path + "/" + file_prefix)
    npy_output_dir_count = 0
    for item in npy_output_dir_contents:
        npy_output_dir_count += 1
    print("Numpy output dir count: " + str(npy_output_dir_count))
    # increment
    npy_output_dir_count += 1

    # check that the output dirs are the same size. (Could cause data to be out of sync)
    if json_output_dir_count != npy_output_dir_count:
        raise ValueError("Discrepency detected between output directory sizes. Exiting....")

    # For each config - do the extraction and conversion
    for i, item in enumerate(config_dir_contents):
        print("Extracting data using: " + item)
        this_name = item.split(".")[0]
        dir_num = i + 1
        new_json_path = json_output_path + "/" + file_prefix + "/" + str(json_output_dir_count) + "/"
        new_json_file = new_json_path + file_prefix + "-" + this_name + "-" + str(json_output_dir_count) + ".json"
        new_npy_path = npy_output_path + "/" + file_prefix + "/" + str(json_output_dir_count) + "/"
        new_npy_file = new_npy_path + file_prefix + "-" + this_name + "-" + str(json_output_dir_count) + ".npy"
        print(new_json_file)
        print(new_npy_file)

        # Call extract keys - [script, nuclear data, a config, json output path + /prefix +
        # subprocess.call(['extract-key-values.py', 'input', 'config', output_path + file_prefix + this_name + file postfix])
        os.makedirs(new_json_path, exist_ok=True)
        subprocess.call(
            [
                "./extract-key-values.py",
                input_file,
                config_dir + "/" + item,
                new_json_file,
            ]
        )
        os.makedirs(new_npy_path, exist_ok=True)
        subprocess.call(
            [
                "./json-to-numpy.py",
                new_json_file,
                new_npy_file,
            ]
        )


if __name__ == "__main__":
    main()
