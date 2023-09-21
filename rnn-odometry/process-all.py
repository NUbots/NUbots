import argparse
import json
import os
import subprocess


def main():
    parser = argparse.ArgumentParser(description="Process nuclear output json files into pieces for training")
    parser.add_argument("config_dir", help="Path to the directory holding the config files")
    parser.add_argument(
        "file_prefix", help="The prefix to add for identifying the type of path represented by the data"
    )
    parser.add_argument("json_output", help="Path to the output directory to hold the processed json data")
    parser.add_argument("npy_output", help="Path to the output directory to hold the processed numpy data")

    args = parser.parse_args()
    config_dir = args.config_dir
    file_prefix = args.file_prefix
    json_output = args.json_output
    npy_output = args.npy_output

    # get the contents of the config directory

    config_dir_contents = os.listdir(config_dir)
    # print contents
    for i, item in enumerate(config_dir_contents):
        print("Extracting data using: " + item)
        this_name = item.split(".")[0]
        dir_num = i + 1

        # Call extract keys - [script, nuclear data, a config, json output path + /prefix +
        # subprocess.call(['extract-key-values.py', ''])

    # Call extract key values FOR EACH config file and direct it to
    # processed-outputs/json/{type of path}/{dir number}


if __name__ == "__main__":
    main()
