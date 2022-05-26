#!/usr/bin/env python3

# Without this block isort will totally mess this up
if True:
    # Make tensorflow quiet unless we tell it to talk
    import os

import cv2
import numpy as np
import tensorflow as tf
from ruamel.yaml import YAML
from tqdm import tqdm, trange

import b

from .camera_calibration.callback import ExtrinsicProgress, IntrinsicProgress
from .camera_calibration.grid_distance import grid_distance
from .camera_calibration.loss import extrinsic_loss
from .camera_calibration.metric import *
from .camera_calibration.model import *

# The dtype we will use to calibrate, 64 bit floats tend to be a little more numerically stable
TF_DTYPE = tf.float64
NP_DTYPE = np.float64


def register(command):
    command.help = "Calculate camera intrinsics using an nbs recordings of asymmetric circles patterns."

    # Command arguments
    command.add_argument(
        "files", metavar="files", nargs="+", help="The nbs files to extract the compressed images from"
    )
    command.add_argument(
        "--config",
        "-c",
        dest="config_path",
        required=True,
        help="The directory containing the configuration files for the cameras",
    )
    command.add_argument("--rows", default=4, type=int, help="the number of rows in the asymmetric circles grid")
    command.add_argument("--cols", default=11, type=int, help="the number of columns in the asymmetric circles grid")
    command.add_argument(
        "--grid_size", default=0.04, type=float, help="the distance between rows/cols in the grid in meters"
    )
    command.add_argument(
        "--no-intrinsics",
        action="store_true",
        help="use the intrinsics provided in the configuration file and do not attempt to optimise them",
    )
    command.add_argument(
        "--patience", default=100, type=int, help="how many epochs of the loss not reducing before we terminate"
    )


def run(files, config_path, cols, rows, grid_size, patience, **kwargs):

    # Disable gpus for camera calibration, CPU does a much better job here due to all the memory shuffling
    tf.config.set_visible_devices([], "GPU")

    # Find all the asymmetric circles grids in the input nbs files
    grids = find_grids(files, cols, rows)

    # Load the configuration files
    yaml = YAML()
    configurations = {}
    for path in os.listdir(config_path):

        if path.endswith(".yaml"):
            name = path[:-5]
            # Open up the camera calibration yaml file for this camera and load the values that are in it as the defaults
            with open(os.path.join(config_path, path), "r") as f:
                configurations[name] = {"config": yaml.load(f)}

    ############################################################
    #  _                    _   __  __           _      _      #
    # | |    ___   __ _  __| | |  \/  | ___   __| | ___| |___  #
    # | |   / _ \ / _` |/ _` | | |\/| |/ _ \ / _` |/ _ \ / __| #
    # | |__| (_) | (_| | (_| | | |  | | (_) | (_| |  __/ \__ \ #
    # |_____\___/ \__,_|\__,_| |_|  |_|\___/ \__,_|\___|_|___/ #
    ############################################################
    grids = {k: v for k, v in grids.items() if k in configurations}
    for name, data in grids.items():
        # Load the configuration for this camera
        config = configurations[name]["config"]

        # Check which projection we are using and load an appropriate model starting with the values in the configuration
        projection = config["lens"]["projection"]
        LensModel = {"EQUIDISTANT": EquidistantModel, "EQUISOLID": EquisolidModel, "RECTILINEAR": RectilinearModel}[
            projection
        ]

        model = LensModel(
            camera_name=name,
            focal_length=config["lens"]["focal_length"],
            centre=config["lens"]["centre"],
            k=config["lens"]["k"],
            grid_size=grid_size,
            inverse_parameters=4,
            dtype=TF_DTYPE,
        )

        # Compile the model
        model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=1e-2), run_eagerly=True)

        # Put the model into the config
        configurations[name]["model"] = model

    #################################################################################
    #    ____      _ _ _               _         ____            _                  #
    #   / ___|__ _| (_) |__  _ __ __ _| |_ ___  / ___| _   _ ___| |_ ___ _ __ ___   #
    #  | |   / _` | | | '_ \| '__/ _` | __/ _ \ \___ \| | | / __| __/ _ \ '_ ` _ \  #
    #  | |__| (_| | | | |_) | | | (_| | ||  __/  ___) | |_| \__ \ ||  __/ | | | | | #
    #   \____\__,_|_|_|_.__/|_|  \__,_|\__\___| |____/ \__, |___/\__\___|_| |_| |_| #
    #                                                  |___/                        #
    #################################################################################
    # Find the minimum starting timestamp to control our update rate and to remove the leading time
    start_time = min([min([v["timestamp"][0] + v["timestamp"][1] * 1e-9 for v in values]) for values in grids.values()])
    end_time = max([max([v["timestamp"][0] + v["timestamp"][1] * 1e-9 for v in values]) for values in grids.values()])
    rate = 1 / 50

    # Get a list of times that we will run on
    times = np.arange(start_time, end_time, rate)

    samples = {}
    resamples = {}
    for name, data in grids.items():
        # Extract all of the point grids, cast them to the correct dtype and stack them into rows/cols
        points = tf.reshape(
            tf.cast(tf.stack([p["centres"] for p in data if p["centres"] is not None], axis=0), TF_DTYPE),
            (-1, rows, cols, 2),
        )
        dimensions = tf.cast(
            tf.stack([p["dimensions"][-2::-1] for p in data if p["centres"] is not None], axis=0), dtype=TF_DTYPE
        )[:, tf.newaxis, tf.newaxis, :]

        # Width normalise the points so all cameras have similar gradients
        points = (dimensions * 0.5 - points) / dimensions[..., 0:1]
        samples[name] = tf.expand_dims(points, 0)

        # Resample at a constant rate for the extrinsics and work out indexes and transforms
        lerp = []
        d_idx = 0  # Data index (where the point is stored)
        r_idx = 0  # Resample index (what index of resampling we are up to
        for c, n in zip(data[:-1], data[1:]):

            # Current and next timestamps
            ct = c["timestamp"][0] + c["timestamp"][1] * 1e-9
            nt = n["timestamp"][0] + n["timestamp"][1] * 1e-9

            # Check if either is actually just none values
            c_valid = c["centres"] is not None
            n_valid = n["centres"] is not None
            invalid = not (c_valid and n_valid)

            # Move through our resample lists adding in points
            while r_idx < len(times) and times[r_idx] <= nt:
                t = times[r_idx]

                # If the timestamp is before our range or one of our centres doesn't exist we are invalid so add an invalid point
                if invalid or t < ct:
                    lerp.append([0, 0, -1.0])
                # Get our two indices and interpolate them
                else:
                    lerp.append([d_idx, d_idx + 1, (t - ct) / (nt - ct)])

                r_idx = r_idx + 1

            # Update our indices
            d_idx = d_idx + c_valid

        # Add in invalid values until we reach the appropriate length
        for i in range(len(lerp), len(times)):
            lerp.append([0, 0, -1.0])

        idx = np.array([l[:2] for l in lerp])
        factor = tf.expand_dims(np.array([l[2] for l in lerp]), -1)[..., tf.newaxis, tf.newaxis]
        resamples[name] = {"idx": idx, "factor": factor}

    # Create our camera system model including intrinsics and extrinsics
    model = CameraSystemModel(
        cameras={k: v["model"] for k, v in configurations.items()},
        Hpcs={k: np.array(configurations[k]["config"]["lens"]["Hpc"], dtype=NP_DTYPE) for k in grids.keys()},
        resamples=resamples,
        dtype=TF_DTYPE,
    )
    model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=1e-4), run_eagerly=True)

    # Train the model
    history = model.fit(
        x=samples,
        epochs=1000000,
        batch_size=1,
        verbose=0,
        callbacks=[
            # 0 Learning rate for the first epoch so that if our current optimisation is the best we don't lose it
            tf.keras.callbacks.LearningRateScheduler(schedule=lambda epoch: 0.0 if epoch == 0 else 1e-2),
            tf.keras.callbacks.EarlyStopping(monitor="loss", patience=patience - 1, restore_best_weights=True),
            Progress([name for name in grids.keys()], patience),
        ],
    )

    # Update all the configurations so we can save
    Hpb = tf.linalg.inv(model.Hbp)
    for name in configurations.keys():

        # Grab the data for this specific camera
        cfg = configurations[name]["config"]
        mdl = model.cameras[name]
        Rbc = compose(model.Rbcs[name])
        rCBb = model.rCBbs[name]

        # Update the intrinsics in the config
        cfg["lens"]["focal_length"] = float(mdl.focal_length.numpy())
        cfg["lens"]["centre"][0] = float(mdl.centre[0].numpy())
        cfg["lens"]["centre"][1] = float(mdl.centre[1].numpy())
        for i in range(len(cfg["lens"]["k"])):
            cfg["lens"]["k"][i] = float(mdl.k[i].numpy())

        # Construct the matrix and reapply the base camera position
        Hbc = tf.tensor_scatter_nd_update(
            tf.pad(tf.concat([Rbc, rCBb[:, tf.newaxis]], axis=-1), [[0, 1], [0, 0]]), [[3, 3]], [1.0]
        )
        Hpc = tf.linalg.matmul(Hpb, Hbc)

        # We have to do it this way so we only update values in ruamel so it keeps formatting
        for i in range(Hpc.shape[0]):
            for j in range(Hpc.shape[1]):
                cfg["lens"]["Hpc"][i][j] = float(Hpc[i][j].numpy())

    # Write out the new model configuration
    for name, config in configurations.items():
        with open(os.path.join(config_path, "{}.yaml".format(name)), "w") as f:
            yaml.dump(config["config"], f)
