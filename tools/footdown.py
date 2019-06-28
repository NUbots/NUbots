#!/usr/bin/env python3

import os
import tensorflow as tf
import yaml
from tqdm import tqdm
from util import nbs_decoder
import numpy as np

SERVO_ID = {
    "R_SHOULDER_PITCH": 0,
    "L_SHOULDER_PITCH": 1,
    "R_SHOULDER_ROLL": 2,
    "L_SHOULDER_ROLL": 3,
    "R_ELBOW": 4,
    "L_ELBOW": 5,
    "R_HIP_YAW": 6,
    "L_HIP_YAW": 7,
    "R_HIP_ROLL": 8,
    "L_HIP_ROLL": 9,
    "R_HIP_PITCH": 10,
    "L_HIP_PITCH": 11,
    "R_KNEE": 12,
    "L_KNEE": 13,
    "R_ANKLE_PITCH": 14,
    "L_ANKLE_PITCH": 15,
    "R_ANKLE_ROLL": 16,
    "L_ANKLE_ROLL": 17,
    "HEAD_YAW": 18,
    "HEAD_PITCH": 19,
}


def displacement(fk):
    return -(
        np.linalg.inv(
            np.array(
                [
                    [fk.x.x, fk.y.x, fk.z.x, fk.t.x],
                    [fk.x.y, fk.y.y, fk.z.y, fk.t.y],
                    [fk.x.z, fk.y.z, fk.z.z, fk.t.z],
                    [fk.x.t, fk.y.t, fk.z.t, fk.t.t],
                ]
            )
        )[2, 3]
    )


def dataset(path, state):
    for type_name, timestamp, msg in tqdm(
        nbs_decoder.decode(path), dynamic_ncols=True, unit="packets"
    ):
        if type_name == "message.input.Sensors":

            # Work out the class
            if state == "WALK":
                l_height = displacement(
                    msg.forward_kinematics[SERVO_ID["L_ANKLE_ROLL"]]
                )
                r_height = displacement(
                    msg.forward_kinematics[SERVO_ID["R_ANKLE_ROLL"]]
                )
                print(path, l_height, r_height)
            elif state == "UP":
                y = 0
            elif state == "DOWN":
                y = 1
            else:
                raise RuntimeError("The state must be UP DOWN or WALK")
            data = (msg.servo[0].present_position, msg.servo[0].present_velocity)


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
