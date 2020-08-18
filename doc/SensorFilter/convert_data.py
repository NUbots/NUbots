#! /usr/bin/env python3

import json
import argparse
import yaml
import numpy as np

parser = argparse.ArgumentParser(description="Convert MATLAB/Simulink exported JSON data to YAML data")
parser.add_argument("--input-file", default="test.json", help="JSON file to import")
parser.add_argument("--output-file", default="test.yaml", help="YAML file to export to")

args = parser.parse_args()

# The field in the data that we need to work on (all of the homogeneous transforms)
# fmt: off
fields = [
    "Htw",
    "HtRSp", "HtLSp",
    "HtRSr", "HtLSr",
    "HtRE",  "HtLE",
    "HtRHy", "HtLHy",
    "HtRHr", "HtLHr",
    "HtRHp", "HtLHp",
    "HtRK",  "HtLK",
    "HtRAp", "HtLAp",
    "HtRAr", "HtLAr",
    "HtHy",  "HtHp",
]
# fmt: on

# Load the JSON data
with open(args.input_file, "r") as f:
    data = json.load(f)

# The homogeneous transforms are stored in the format WxHxN
# Transpose them to be in the format NxHxW
for field in fields:
    data[field] = np.transpose(data[field], axes=[2, 1, 0]).tolist()

# Save the yaml data
with open(args.output_file, "w") as f:
    yaml.dump(data, f, width=120)
