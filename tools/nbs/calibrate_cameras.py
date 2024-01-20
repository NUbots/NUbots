#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2020 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import bisect
import copyreg
import json
import math
import multiprocessing
import os
import pickle

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
from .images import decode_image, fourcc
from utility.nbs import LinearDecoder as Decoder

# The dtype we will use to calibrate, 64 bit floats tend to be a little more numerically stable
TF_CALIBRATION_DTYPE = tf.float64
NP_CALIBRATION_DTYPE = np.float64

# This code here is needed for OpenCV to work with multiprocessing. When finding the grids we use all the cpu cores
# available. However to do this we need to be able to send the OpenCV data back and because of the way that the OpenCV
# module works, pickling doesn't work properly by default as the class is a function. By remappping it here we can fix
# the problem so that it can be pickled properly
def _pickle_keypoints(point):
    return cv2.KeyPoint, (*point.pt, point.size, point.angle, point.response, point.octave, point.class_id)


copyreg.pickle(cv2.KeyPoint().__class__, _pickle_keypoints)


def register(command):
    command.help = "Calculate camera intrincs using an nbs recordings of checkered board patterns."

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
        "--no-extrinsics",
        action="store_true",
        help="use the intrinsics provided in the configuration file and do not attempt to optimise them",
    )


def packetise_stream(decoder):
    for packet in decoder:
        # Check for compressed images
        if packet.type.name == "message.output.CompressedImage":
            # Get some useful info into a pickleable format
            yield {
                "camera_name": packet.msg.name,
                "timestamp": (packet.msg.timestamp.seconds, packet.msg.timestamp.nanos),
                "data": packet.msg.data,
                "format": packet.msg.format,
            }



def process_frame(item, rows, cols):
    data = decode_image(item["data"], item["format"])

    if len(data) == 1:
        img = data[0]["image"].numpy()
        fmt = data[0]["fourcc"]
    else:
        img = [d for d in data if d["name"] == "_colour"][0]["image"].numpy()
        fmt = data[0]["fourcc"]

    # Debayer if we need to
    if fmt == fourcc("BGGR"):
        img = cv2.cvtColor(img, cv2.COLOR_BayerBG2RGB)
    elif fmt == fourcc("RGGB"):
        img = cv2.cvtColor(img, cv2.COLOR_BayerRG2RGB)
    elif fmt == fourcc("GRBG"):
        img = cv2.cvtColor(img, cv2.COLOR_BayerGR2RGB)
    elif fmt == fourcc("GBRG"):
        img = cv2.cvtColor(img, cv2.COLOR_BayerGB2RGB)

    # The detector is based on OpenCVs blob detector to understand what some of these parameters mean, read this
    # https://www.learnopencv.com/blob-detection-using-opencv-python-c/
    params = cv2.SimpleBlobDetector_Params()

    params.minThreshold = 10
    params.maxThreshold = 200
    params.thresholdStep = 10

    params.minDistBetweenBlobs = 5

    params.filterByConvexity = True
    params.minConvexity = 0.95

    # Filter by area going from a circle radius of about 2 pixels to one of about 1/22th of the width
    params.filterByArea = True
    params.minArea = 15
    params.maxArea = math.pi * (img.shape[0] * (1 / 22)) ** 2

    params.filterByCircularity = False

    params.filterByInertia = False

    detector = cv2.SimpleBlobDetector_create(params)

    # If we failed normally, try with clustering as it often does better near the edges
    ret, centres = cv2.findCirclesGrid(img, (rows, cols), flags=cv2.CALIB_CB_ASYMMETRIC_GRID, blobDetector=detector)
    if not ret:
        ret, centres = cv2.findCirclesGrid(
            img, (rows, cols), flags=cv2.CALIB_CB_ASYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING, blobDetector=detector
        )

    return {
        "name": item["camera_name"],
        "timestamp": item["timestamp"],
        "image": img,
        "blobs": detector.detect(img),
        "centres": centres if ret else None,
    }


def find_grids(files, rows, cols):

    # Work out what the name of the pickle file will be for caching
    pickle_path = "{}.pickle".format(
        "_".join(os.path.basename(f).replace(".nbs", "").replace(".nbz", "").replace(".gz", "") for f in files)
    )

    # The points from the images that have been gathered
    if not os.path.isfile(pickle_path):
        # Read the nbs file
        with multiprocessing.Pool(multiprocessing.cpu_count()) as pool:
            decoder = Decoder(*files)

            grids = {}

            print("Detecting asymmetric circles grids")

            with tqdm(total=len(decoder), unit="B", unit_scale=True, dynamic_ncols=True) as progress:

                results = []

                # Function that updates the results
                def update_results(msg):

                    # Update the progress based on the image we are up to
                    # progress.update(msg["bytes_read"])

                    img = msg["image"]
                    img = cv2.drawKeypoints(
                        img, msg["blobs"], np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
                    )
                    if msg["centres"] is not None:
                        img = cv2.drawChessboardCorners(img, (rows, cols), msg["centres"], True)

                    # cv2.imshow(msg["name"], img)
                    # cv2.waitKey(1)

                    # Add this to the grids list
                    if msg["name"] not in grids:
                        grids[msg["name"]] = []
                    grids[msg["name"]].append(
                        {
                            "timestamp": msg["timestamp"],
                            # Normalise the pixel coordinates to be based from the centre of the image
                            # And then divide by the width of the image to get a normalised coordinate
                            "centres": None
                            if msg["centres"] is None
                            else (np.array(img.shape[:2][::-1], dtype=NP_CALIBRATION_DTYPE) * 0.5 - msg["centres"]) / img.shape[1],
                            "dimensions": img.shape,
                        }
                    )

                for msg in packetise_stream(decoder):
                    # Add a task to the pool to process
                    results.append(pool.apply_async(process_frame, (msg, rows, cols)))

                    # Only buffer a number images for each cpu core to avoid running out of memory
                    if len(results) > 128 * multiprocessing.cpu_count():
                        results[0].wait()

                    # If the next one is ready process it
                    if len(results) > 0 and results[0].ready():
                        update_results(results.pop(0).get())

                while len(results) > 0:
                    update_results(results.pop(0).get())

                with open(pickle_path, "wb") as f:
                    pickle.dump(grids, f, protocol=pickle.HIGHEST_PROTOCOL)

    # Close all the windows
    # cv2.destroyAllWindows()

    # # If we have a pickle file load it
    else:
        # Load the pickle file
        with open(pickle_path, "rb") as f:
            grids = pickle.load(f)

    # Sort the grids by timestamp
    for v in grids.values():
        v.sort(key=lambda v: v["timestamp"])

    return grids


# Measures the quality in the Axis using SVD, i.e. rectangularity and the flatness of the detected grid.
# We want to discard the nosiest samples of the dataset, and discard data with percentage error greater than allowed_error.
def plane_quality(points, rows, cols, grid_size):
    # Work out our ideal SVD values
    world_grid = np.empty([cols, rows, 2], dtype=NP_CALIBRATION_DTYPE)

    # Set the coordinate values in our grid
    world_grid[:, :, 0] = (np.arange(cols) * grid_size)[:, np.newaxis]
    world_grid[0::2, :, 1] = np.arange(rows) * (grid_size * 2)
    world_grid[1::2, :, 1] = np.arange(rows) * (grid_size * 2) + grid_size
    world_grid = tf.convert_to_tensor(world_grid)
    world_grid = tf.reshape(world_grid - tf.reduce_mean(world_grid, axis=[0, 1], keepdims=True), (-1, 2))

    # Allowed x/y percentage error
    allowed_error = 0.02
    max_xy = tf.linalg.svd(tf.transpose((1 + allowed_error) * world_grid), compute_uv=False)
    min_xy = tf.linalg.svd(tf.transpose((1 - allowed_error) * world_grid), compute_uv=False)

    # Mean centre
    centred = points - tf.reduce_mean(points, axis=[1, 2], keepdims=True)

    # Flatten into planes per detection
    centred = tf.transpose(tf.reshape(centred, (-1, tf.multiply(*centred.shape[1:-1]), 3)), [0, 2, 1])

    # We run out of GPU memory if we try to do this all at once since svd is pretty memory heavy so instead we split up
    # the task and do it in segments
    skip = 1000
    result = tf.concat(
        [
            tf.linalg.svd(centred[i : min(i + skip, centred.shape[0])], compute_uv=False)
            for i in range(0, centred.shape[0], skip)
        ],
        axis=0,
    )

    # Z threshold should be flat and ignore the highest 20% of the values for std calculations
    z_threshold = tf.math.reduce_mean(result[:, 2]) + tf.math.reduce_std(
        tf.sort(result[:, 2])[: (80 * result.shape[0]) // 100]
    )

    xy_valid = tf.logical_and(min_xy <= result[:, 0:2], result[:, 0:2] <= max_xy)
    z_valid = result[:, 2] < z_threshold

    return tf.squeeze(tf.where(tf.logical_and(tf.reduce_all(xy_valid[:, :2], axis=-1), z_valid)), axis=-1)


def run(files, config_path, rows, cols, grid_size, no_intrinsics, no_extrinsics, **kwargs):
    yaml = YAML()
    # Load all the grids
    grids = find_grids(files, rows, cols)

    # Load the configuration files
    configurations = {}
    for path in os.listdir(config_path):

        if path.endswith(".yaml"):
            name = path[:-5]
            # Open up the camera calibration yaml file for this camera and load the values that are in it as the defaults
            with open(os.path.join(config_path, path), "r") as f:
                configurations[name] = {"config": yaml.load(f)}

    # Calculate the intrinsics for each of the lenses
    for name, data in grids.items():

        if name not in configurations:
            raise RuntimeError("There is no file in the configuration directory for the {} camera".format(name))

        print("Loading intrinsics for the {} camera".format(name))

        # Load the configuration for this camera
        config = configurations[name]["config"]

        # Check which projection we are using and load an appropriate model starting with the values in the configuration
        projection = config["lens"]["projection"]
        if projection == "EQUIDISTANT":
            LensModel = EquidistantModel
        if projection == "EQUISOLID":
            LensModel = EquisolidModel
        if projection == "RECTILINEAR":
            LensModel = RectilinearModel

        model = LensModel(
            focal_length=config["lens"]["focal_length"],
            centre=config["lens"]["centre"],
            k=config["lens"]["k"],
            inverse_parameters=4,
            dtype=TF_CALIBRATION_DTYPE,
        )

        # Compile the model
        # For computers with a smaller GPU, run_eagerly should be set to True.
        model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=1e-2))

        # Put the model into the config
        configurations[name]["model"] = model

        # Assume the resolution is the same across all the images, this lets us get values in pixels later
        dimensions = data[0]["dimensions"]

        if not no_intrinsics:
            # Extract all of the point grids that were found into a single matrix
            points = tf.stack([p["centres"] for p in data if p["centres"] is not None], axis=0)

            # Stack up the points into rows/cols
            points = tf.cast(tf.reshape(points, (points.shape[0], cols, rows, 2)), dtype=TF_CALIBRATION_DTYPE)

            # If not empty, calibrate and write new config values
            if tf.size(points) != 0:
                history = model.fit(
                    x=points,
                    # y=tf.ones([*points.shape[:-1], 3], dtype=TF_CALIBRATION_DTYPE),
                    epochs=1000000,
                    batch_size=points.shape[0],
                    verbose=0,
                    callbacks=[
                        # 0 Learning rate for the first epoch so that if our current optimisation is the best we don't lose it
                        tf.keras.callbacks.LearningRateScheduler(schedule=lambda epoch: 0.0 if epoch == 0 else 1e-2),
                        tf.keras.callbacks.EarlyStopping(monitor="loss", patience=99, restore_best_weights=True),
                        IntrinsicProgress(),
                    ],
                )

                # Update the intrinsics in the config
                config["lens"]["focal_length"] = float(model.focal_length.numpy())
                config["lens"]["centre"][0] = float(model.centre[0].numpy())
                config["lens"]["centre"][1] = float(model.centre[1].numpy())
                for i in range(len(config["lens"]["k"])):
                    config["lens"]["k"][i] = float(model.k[i].numpy())

                # Work out how much the loss has improved by
                best_idx = np.argmin(history.history["loss"])

                # Extract the angular errors and express them in pixels also
                collinearity = math.sqrt(history.history["collinearity"][best_idx])
                parallelity = math.sqrt(history.history["parallelity"][best_idx])
                orthogonality = math.sqrt(history.history["orthogonality"][best_idx])
                collinearity_px = model.r(tf.cast(collinearity, model.dtype)) * dimensions[1]
                parallelity_percent = 100 * math.tan(parallelity)
                orthogonality_percent = 100 * math.tan(orthogonality)

                inverse_qualities = [math.sqrt(model.inverse_quality(i).numpy()) for i in range(1, 10)]

                print("Intrinsic calibration results for {} camera".format(name))
                print("\t ƒ: {:.3f}".format(model.focal_length.numpy()))
                print("\tΔc: [{}]".format(", ".join(["{:+.3f}".format(v) for v in model.centre.numpy()])))
                print("\t k: [{}]".format(", ".join(["{:+.3f}".format(v) for v in model.k.numpy()])))
                print("\tik: [{}]".format(", ".join(["{:+.3f}".format(v) for v in model.inverse_coeffs()])))
                print("Error")
                print("\t ↔: {:.3f}º ({:.3f}px) ".format(collinearity * 180 / math.pi, collinearity_px))
                print("\t||: {:.3f}º ({:.3f}%)".format(parallelity * 180 / math.pi, parallelity_percent))
                print("\t ⟂: {:.3f}º ({:.3f}%)".format(orthogonality * 180 / math.pi, orthogonality_percent))
                print(
                    "\t k: [{}]".format(
                        ", ".join(["{:.2f}px ({:.2f}%)".format(v * dimensions[1], v * 100) for v in inverse_qualities])
                    )
                )

                original_loss = history.history["loss"][0]
                best_loss = min(history.history["loss"])
                print()
                print("Loss improved by {:.2g}%".format(100.0 * (original_loss - best_loss) / original_loss))
                print()

                # Only need to write new cfgs if we have data for calibration
                # Write out the new model configuration
                for name, config in configurations.items():
                    with open(os.path.join(config_path, "{}.yaml".format(name)), "w") as f:
                        yaml.dump(config["config"], f)

    # Create the extrinsics dataset
    if not no_extrinsics:

        print("Calibrating extrinsics")

        # Extract the timestamps for each so we can work out which is the best baseline camera
        timestamps = {name: [(*v["timestamp"], v["centres"] is not None) for v in data] for name, data in grids.items()}

        with tqdm(total=9, unit="steps", dynamic_ncols=True, leave=False) as total_progress:
            # Create the values for each of the elements
            total_progress.set_description("Gathering pairs")
            extrinsic_data = {(a, b): list() for a in grids for b in grids if a != b}
            with tqdm(total=len(grids) * (len(grids) - 1), unit=" pairs", dynamic_ncols=True, leave=False) as progress:
                for n_1, d_1 in grids.items():
                    for n_2, d_2 in grids.items():
                        if n_1 != n_2:
                            for v_1 in d_1:
                                if v_1["centres"] is not None:
                                    progress.set_description("{}->{}".format(n_1, n_2))
                                    # Check if our timestamp is contained within the range of the other camera
                                    if d_2[0]["timestamp"] <= v_1["timestamp"] < d_2[-1]["timestamp"]:

                                        # Do a linear interpolation of the points based on the time
                                        hi = bisect.bisect_right(timestamps[n_2], (*v_1["timestamp"], True))
                                        lo = hi - 1

                                        # Get the two points
                                        v_2a = d_2[lo]
                                        v_2b = d_2[hi]

                                        # Check if these are both valid
                                        if v_2a["centres"] is not None and v_2b["centres"] is not None:

                                            # Calculate the linear interpolation factor
                                            factor = (
                                                (v_1["timestamp"][0] - v_2a["timestamp"][0])
                                                + (v_1["timestamp"][1] - v_2a["timestamp"][1]) * 1e-9
                                            ) / (
                                                (v_2b["timestamp"][0] - v_2a["timestamp"][0])
                                                + (v_2b["timestamp"][1] - v_2a["timestamp"][1]) * 1e-9
                                            )

                                            extrinsic_data[(n_1, n_2)].append(
                                                (v_1["centres"], v_2a["centres"], v_2b["centres"], factor)
                                            )

                            progress.update()

            # Progress bar update
            total_progress.update()
            total_progress.set_description("Tensorifying")
            # /Progress bar update

            # Put all the values into tensors
            extrinsic_data = {
                k: (
                    tf.reshape(np.array([v[0] for v in data], dtype=np.float64), (len(data), cols, rows, 2)),
                    tf.reshape(np.array([v[1] for v in data], dtype=np.float64), (len(data), cols, rows, 2)),
                    tf.reshape(np.array([v[2] for v in data], dtype=np.float64), (len(data), cols, rows, 2)),
                    tf.convert_to_tensor(np.array([v[3] for v in data], dtype=np.float64)),
                )
                for k, data in tqdm(extrinsic_data.items(), unit=" pairs", dynamic_ncols=True, leave=False)
            }

            # Progress bar update
            total_progress.update()
            total_progress.set_description("Running Intrinsics")
            # /Progress bar update

            # Apply the intrinsic parameters to the pixel coordinates to get unit vectors
            extrinsic_data = {
                k: (
                    configurations[k[0]]["model"].predict(v[0]),
                    configurations[k[1]]["model"].predict(v[1]),
                    configurations[k[1]]["model"].predict(v[2]),
                    v[3],
                )
                for k, v in tqdm(extrinsic_data.items(), unit=" pairs", dynamic_ncols=True, leave=False)
            }

            # Progress bar update
            total_progress.update()
            total_progress.set_description("Estimating Distance")
            # /Progress bar update

            # Calculate the distance to every point in every grid so we can compare them to the distances
            extrinsic_data = {
                k: (
                    v[0],
                    tf.add(
                        tf.multiply(v[1], tf.subtract(1.0, v[3][:, tf.newaxis, tf.newaxis, tf.newaxis])),
                        tf.multiply(v[2], v[3][:, tf.newaxis, tf.newaxis, tf.newaxis]),
                    ),
                    grid_distance(v[0], rows, cols, grid_size, k[0]),
                    tf.add(
                        tf.multiply(
                            grid_distance(v[1], rows, cols, grid_size, k[1]),
                            tf.subtract(1.0, v[3])[:, tf.newaxis, tf.newaxis],
                        ),
                        tf.multiply(grid_distance(v[2], rows, cols, grid_size, k[1]), v[3][:, tf.newaxis, tf.newaxis]),
                    ),
                )
                for k, v in tqdm(extrinsic_data.items(), unit=" pairs", dynamic_ncols=True, leave=False)
            }

            # Progress bar update
            total_progress.update()
            total_progress.set_description("Estimating Quality")
            # /Progress bar update

            # Get the quality of each plane for the extrinsic data to do throwouts
            extrinsic_data = {
                k: (
                    *v,
                    tf.squeeze(
                        tf.sparse.to_dense(
                            tf.sets.intersection(
                                plane_quality(v[0] * v[2][..., tf.newaxis], rows, cols, grid_size)[tf.newaxis],
                                plane_quality(v[1] * v[3][..., tf.newaxis], rows, cols, grid_size)[tf.newaxis],
                            )
                        ),
                        axis=0,
                    ),
                )
                for k, v in tqdm(extrinsic_data.items(), unit=" pairs", dynamic_ncols=True, leave=False)
            }

            # Progress bar update
            total_progress.update()
            total_progress.set_description("Discarding")
            # /Progress bar update

            # Gather the indices that were selected by plane quality
            extrinsic_data = {
                k: (tf.gather(v[0], v[4]), tf.gather(v[1], v[4]), tf.gather(v[2], v[4]), tf.gather(v[3], v[4]))
                for k, v in tqdm(extrinsic_data.items(), unit=" pairs", dynamic_ncols=True, leave=False)
            }

            # Progress bar update
            total_progress.update()
            total_progress.set_description("Weighting Samples")
            # /Progress bar update

            # Calculate the per point weight by using the relative distance of the grid points
            sample_weights = []
            for v in tqdm(extrinsic_data.values(), unit=" pairs", dynamic_ncols=True, leave=False):
                # Calculate the mean position of the grid
                g1 = tf.reduce_mean(v[0] * v[2][..., tf.newaxis], axis=[1, 2])
                g2 = tf.reduce_mean(v[1] * v[3][..., tf.newaxis], axis=[1, 2])

                # Get a factor of how unique each point is
                g1_unique = tf.reduce_mean(tf.linalg.norm(g1[tf.newaxis :, :] - g1[:, tf.newaxis], axis=-1), axis=-1)
                g2_unique = tf.reduce_mean(tf.linalg.norm(g2[tf.newaxis :, :] - g2[:, tf.newaxis], axis=-1), axis=-1)

                # Uniqueness combination, how unique is this pair
                unique = tf.divide(g1_unique, tf.reduce_sum(g1_unique)) * tf.divide(g2_unique, tf.reduce_sum(g2_unique))
                unique = tf.multiply(tf.divide(unique, tf.reduce_sum(unique)), unique.shape[0])

                taxi_distance = tf.square(tf.linalg.norm(g1, axis=-1)) + tf.square(tf.linalg.norm(g2, axis=-1))
                distance_weight = tf.math.reciprocal(
                    tf.multiply(tf.divide(taxi_distance, tf.reduce_sum(taxi_distance)), taxi_distance.shape[0])
                )
                distance_weight = tf.multiply(
                    tf.divide(distance_weight, tf.reduce_sum(distance_weight)), distance_weight.shape[0]
                )

                sample_weight = unique * distance_weight
                sample_weight = tf.multiply(
                    tf.divide(sample_weight, tf.reduce_sum(sample_weight)), sample_weight.shape[0]
                )

                sample_weights.append(sample_weight)
            sample_weights = tf.concat(sample_weights, axis=-1)

            # Progress bar update
            total_progress.update()
            total_progress.set_description("Weighting Classes")
            # /Progress bar update

            # Calculate the class weighting so each camera pair has equal weight in the sampling
            n_samples = sum([v[0].shape[0] for v in extrinsic_data.values()])
            class_weights = tf.concat(
                [
                    np.full(
                        shape=v[0].shape[0],
                        fill_value=(n_samples / (v[0].shape[0] * len(extrinsic_data))),
                        dtype=NP_CALIBRATION_DTYPE,
                    )
                    for v in extrinsic_data.values()
                ],
                axis=-1,
            )

            # Progress bar update
            total_progress.update()
            total_progress.set_description("Preparing data")
            # /Progress bar update

            # Weights are the combination of the two
            weights = tf.multiply(sample_weights, class_weights)

            # Grab all the Hpcs and make mappings from ids to names
            Hpcs = [
                (k, np.array(configurations[k]["config"]["lens"]["Hpc"], dtype=np.float64))
                for k in sorted(grids.keys())
            ]
            id_for_name = {v[0]: i for i, v in enumerate(Hpcs)}
            name_for_id = {i: v[0] for i, v in enumerate(Hpcs)}
            Hpcs = [v[1] for v in Hpcs]

            # Create the final dataset
            i_0 = tf.concat(
                [
                    np.full(shape=v[0].shape[0], fill_value=id_for_name[k[0]], dtype=np.int32)
                    for k, v in extrinsic_data.items()
                ],
                axis=-1,
            )
            i_1 = tf.concat(
                [
                    np.full(shape=v[1].shape[0], fill_value=id_for_name[k[1]], dtype=np.int32)
                    for k, v in extrinsic_data.items()
                ],
                axis=-1,
            )
            x_0 = tf.concat([v[0] for v in extrinsic_data.values()], axis=0)
            x_1 = tf.concat([v[1] for v in extrinsic_data.values()], axis=0)

            # "Truth" data
            d_0 = tf.concat([v[2] for v in extrinsic_data.values()], axis=0)
            d_1 = tf.concat([v[3] for v in extrinsic_data.values()], axis=0)

            # Progress bar update
            total_progress.update()
            total_progress.set_description("Done")
            # /Progress bar update

        model = ExtrinsicCluster(Hpcs, dtype=TF_CALIBRATION_DTYPE)

        model.compile(
            optimizer=tf.keras.optimizers.Adam(learning_rate=1e-2),
            loss=extrinsic_loss,
            metrics=[alignment, relative_distance, absolute_distance],
        )

        history = model.fit(
            x=[i_0, x_0, i_1, x_1],
            y=tf.stack([d_0, d_1, tf.zeros_like(d_0)], axis=-1),
            epochs=1000000,
            batch_size=4096,
            sample_weight=weights,
            shuffle=True,
            verbose=0,
            callbacks=[
                # 0 Learning rate for the first epoch so that if our current optimisation is the best we don't lose it
                tf.keras.callbacks.LearningRateScheduler(schedule=lambda epoch: 0.0 if epoch == 0 else 1e-2),
                tf.keras.callbacks.EarlyStopping(monitor="loss", patience=20, restore_best_weights=True),
                ExtrinsicProgress(),
            ],
        )

        # Build matrices from our calibration and convert them back to platform space
        Roc = model.compose(tf.stack([[0, 0, 0], *model.rotations], axis=0))
        rCOo = tf.stack([[0, 0, 0], *model.translations], axis=0)
        Hpo = tf.linalg.inv(model.Hc_0p)
        Hoc = tf.pad(tf.concat([Roc, rCOo[:, :, tf.newaxis]], axis=-1), [[0, 0], [0, 1], [0, 0]]) + [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 1],
        ]
        Hpc = tf.einsum("ij,ajk->aik", Hpo, Hoc)

        # Write out the new extrinsic calibration
        for name, config in configurations.items():
            H = Hpc[id_for_name[name]].numpy().tolist()

            # We have to do it this way so we only update values in ruamel so it keeps formatting
            for i in range(Hpc.shape[1]):
                for j in range(Hpc.shape[2]):
                    config["config"]["lens"]["Hpc"][i][j] = H[i][j]

            # Save to a file
            with open(os.path.join(config_path, "{}.yaml".format(name)), "w") as f:
                yaml.dump(config["config"], f)

        # Print out statistics about the optimisation per camera
        for i in range(len(id_for_name)):
            for j in range(i + 1, len(id_for_name)):
                a_idx = tf.squeeze(tf.where(tf.logical_and(i_0 == i, i_1 == j)), axis=1)
                b_idx = tf.squeeze(tf.where(tf.logical_and(i_0 == j, i_1 == i)), axis=1)

                a_pred = model.predict(
                    [tf.gather(i_0, a_idx), tf.gather(x_0, a_idx), tf.gather(i_1, a_idx), tf.gather(x_1, a_idx)]
                )
                b_pred = model.predict(
                    [tf.gather(i_0, b_idx), tf.gather(x_0, b_idx), tf.gather(i_1, b_idx), tf.gather(x_1, b_idx)]
                )

                a_true = tf.stack([tf.gather(d_0, a_idx), tf.gather(d_1, a_idx)], axis=-1)
                b_true = tf.stack([tf.gather(d_0, b_idx), tf.gather(d_1, b_idx)], axis=-1)

                # Calculate our error measures
                absolute_distance_error = 0.5 * (
                    tf.reduce_mean(tf.abs(a_pred[..., :2] - a_true)) + tf.reduce_mean(tf.abs(b_pred[..., :2] - b_true))
                )
                relative_distance_error = 0.5 * (
                    tf.reduce_mean(tf.abs(a_pred[..., :2] - a_true) / a_true)
                    + tf.reduce_mean(tf.abs(b_pred[..., :2] - b_true) / b_true)
                )
                alignment_error = 0.5 * (
                    tf.reduce_mean(tf.acos(tf.clip_by_value(a_pred[..., 2], -1, 1))) * (180.0 / math.pi)
                    + tf.reduce_mean(tf.acos(tf.clip_by_value(b_pred[..., 2], -1, 1))) * (180.0 / math.pi)
                )

                relative_position = Hpc[j, 0:3, -1] - Hpc[i, 0:3, -1]

                print("{}->{}:".format(name_for_id[i], name_for_id[j]))
                print("\tAbsolute Distance Error: {:.2f}m".format(absolute_distance_error))
                print("\tRelative Distance Error: {:.2f}%".format(relative_distance_error * 100))
                print("\tAlignment Error:         {:.2f}º".format(alignment_error))
                print(
                    "\tRelative Position:       ||{:.3f}, {:.3f}, {:.3f}|| = {:.3f}m".format(
                        *relative_position, tf.linalg.norm(relative_position)
                    )
                )
