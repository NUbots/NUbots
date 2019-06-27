#!/usr/bin/env python3

import os
import tensorflow as tf
import yaml
from tqdm import tqdm
from util import nbs_decoder


def dataset(path, state):
    for type_name, timestamp, msg in tqdm(
        nbs_decoder.decode(path), dynamic_ncols=True, unit="packets"
    ):
        if type_name == "message.input.Sensors":
            pass


def register(command):

    # Install help
    command.help = "Train a foot down network using sensor data from the legs"

    # Drone arguments
    command.add_argument(
        "config",
        metavar="config",
        help="The configuration file specifying groups of data files",
    )


def run(config, **kwargs):

    # Load our configuration file to get the individual nbs files
    data_path = os.path.dirname(config)
    with open(config, "r") as f:
        config = yaml.load(f)

    print("Loading data from NBS files")
    for group in tqdm(config["data"], dynamic_ncols=True, unit="group"):
        for f in tqdm(group["files"], dynamic_ncols=True, unit="file"):
            data = dataset(os.path.join(data_path, f), group["state"])
