#!/usr/bin/env python3

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
from tqdm import tqdm

import b

from .camera_calibration.callback import EpochProgress
from .camera_calibration.loss import ExtrinsicLoss, euclidean_error, euclidean_loss
from .camera_calibration.metric import GridError, alignment, collinearity, orthogonality, parallelity
from .camera_calibration.model import Equidistant, Equisolid, ExtrinsicCluster, Rectilinear
from .decoder import Decoder
from .images import decode_image

ROWS = 4
COLS = 11


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

    bytes_read = 0
    for packet in decoder:

        # Check for compressed images
        if packet.type == "message.output.CompressedImage":

            # Work out how many bytes we have read to get to this message
            read = decoder.bytes_read() - bytes_read
            bytes_read = decoder.bytes_read()

            # Get some useful info into a pickleable format
            yield {
                "bytes_read": read,
                "camera_name": packet.msg.name,
                "timestamp": (packet.msg.timestamp.seconds, packet.msg.timestamp.nanos),
                "data": packet.msg.data,
                "format": packet.msg.format,
            }


def process_frame(item):
    data = decode_image(item["data"], item["format"])

    if len(data) == 1:
        img = data[0]["image"].numpy()
    else:
        img = [d for d in data if d["name"] == "_colour"][0]["image"].numpy()

    # TODO need to handle this better
    img = cv2.cvtColor(img, cv2.COLOR_BayerBG2RGB)

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
    ret, centres = cv2.findCirclesGrid(img, (ROWS, COLS), flags=cv2.CALIB_CB_ASYMMETRIC_GRID, blobDetector=detector)
    if not ret:
        ret, centres = cv2.findCirclesGrid(
            img, (ROWS, COLS), flags=cv2.CALIB_CB_ASYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING, blobDetector=detector
        )

    return {
        "name": item["camera_name"],
        "timestamp": item["timestamp"],
        "bytes_read": item["bytes_read"],
        "image": img,
        "blobs": detector.detect(img),
        "centres": centres if ret else None,
    }


def find_grids(files):

    # Work out what the name of the pickle file will be for caching
    pickle_path = "{}.pickle".format(
        "_".join(os.path.basename(f).replace(".nbs", "").replace(".nbz", "").replace(".gz", "") for f in files)
    )

    # The points from the images that have been gathered
    if not os.path.isfile(pickle_path):

        # Read the nbs file
        pool = multiprocessing.Pool(multiprocessing.cpu_count())
        decoder = Decoder(*files)

        grids = {}

        print("Detecting asymmetric circles grids")

        with tqdm(total=len(decoder), unit="B", unit_scale=True, dynamic_ncols=True) as progress:

            results = []

            # Function that updates the results
            def update_results(msg):

                # Update the progress based on the image we are up to
                progress.update(msg["bytes_read"])

                img = msg["image"]
                img = cv2.drawKeypoints(
                    img, msg["blobs"], np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
                )
                if msg["centres"] is not None:
                    img = cv2.drawChessboardCorners(img, (ROWS, COLS), msg["centres"], True)

                cv2.imshow(msg["name"], img)
                cv2.waitKey(1)

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
                        else (np.array(img.shape[:2][::-1], dtype=np.float) * 0.5 - msg["centres"]) / img.shape[1],
                    }
                )

            for msg in packetise_stream(decoder):
                # Add a task to the pool to process
                results.append(pool.apply_async(process_frame, (msg,)))

                # Only buffer 1024 images for each cpu core to avoid running out of memory
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
        cv2.destroyAllWindows()

    # If we have a pickle file load it
    else:
        # Load the pickle file
        with open(pickle_path, "rb") as f:
            grids = pickle.load(f)

    # Sort the grids by timestamp
    for v in grids.values():
        v.sort(key=lambda v: v["timestamp"])

    return grids


def run(files, config_path, no_intrinsics, no_extrinsics, **kwargs):
    yaml = YAML()
    # Load all the grids
    grids = find_grids(files)

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
            model = Equidistant(**config["lens"])
        if projection == "EQUISOLID":
            model = Equisolid(**config["lens"])
        if projection == "RECTILINEAR":
            model = Rectilinear(**config["lens"])

        # Put the model into the config
        configurations[name]["model"] = model

        # Extract all of the point grids that were found into a single matrix
        points = tf.stack([p["centres"] for p in data if p["centres"] is not None], axis=0)

        # Stack up the points into rows/cols
        points = tf.reshape(points, (points.shape[0], COLS, ROWS, 2))

        # Run a prediction to help identify outliers to be removed
        # Even though this is running on an untrained model at this point, the outliers are pretty obvious
        predictions = euclidean_error(tf.convert_to_tensor(model.predict(points)))

        # Gather the samples best losses from each of the error categories and only include the intersection of them
        _, indices = tf.math.top_k(tf.math.negative(predictions), k=(9 * predictions[0].shape[0]) // 10)
        indices = tf.squeeze(
            tf.sparse.to_dense(tf.sets.intersection(tf.sets.intersection(indices[0:1], indices[1:2]), indices[2:3])),
            axis=0,
        )

        # Regather these points
        points = tf.gather(points, indices, batch_dims=0)

        # Average pixel coordinates of the grid
        position = tf.math.reduce_mean(points, axis=[1, 2])
        # Work out the average difference from this point to all the other points
        position_distance = tf.stack(
            [tf.reduce_mean(tf.sqrt(tf.reduce_sum(tf.math.squared_difference(position, p)))) for p in position], axis=0
        )
        position_distance = tf.divide(position_distance, tf.reduce_sum(position_distance))

        # Take the 4 Corner Points: P1,P2,P3,P4 of the Quadrilateral
        P1 = points[:, 0, 0, :]
        P2 = points[:, 0, 3, :]
        P3 = points[:, 10, 3, :]
        P4 = points[:, 10, 0, :]

        # Using the Shoelace formula: https://math.stackexchange.com/questions/1259094/coordinate-geometry-area-of-a-quadrilateral
        area = 0.5 * tf.abs(
            P1[:, 0] * P2[:, 1]
            + P2[:, 0] * P3[:, 1]
            + P3[:, 0] * P4[:, 1]
            + P4[:, 0] * P1[:, 1]
            - P2[:, 0] * P1[:, 1]
            - P3[:, 0] * P2[:, 1]
            - P4[:, 0] * P3[:, 1]
            - P1[:, 0] * P4[:, 1]
        )

        # Area distance is already squared, get the difference of squares and then square root
        area_distance = tf.stack([tf.reduce_mean(tf.sqrt(tf.abs(tf.math.subtract(area, a)))) for a in area], axis=0)
        area_distance = tf.divide(area_distance, tf.reduce_sum(area_distance))

        # Multiply position weight with area weight and then multiply by normalised area (bigger areas get more weight)
        area_weight = tf.sqrt(area)
        weight = tf.multiply(
            tf.multiply(position_distance, area_distance), tf.divide(area_weight, tf.reduce_sum(area_weight))
        )
        weight = tf.multiply(tf.divide(weight, tf.reduce_sum(weight)), weight.shape[0])

        model.compile(
            run_eagerly=True,
            optimizer=tf.keras.optimizers.Adam(learning_rate=1e-2),
            loss=euclidean_loss,
            metrics=[collinearity, parallelity, orthogonality],
        )

        if not no_intrinsics:
            history = model.fit(
                x=points,
                y=tf.ones([*points.shape[:-1], 3], dtype=tf.float32),
                epochs=1000000,
                batch_size=points.shape[0],
                sample_weight=weight,
                verbose=0,
                callbacks=[
                    # 0 Learning rate for the first epoch so that if our current optimisation is the best we don't lose it
                    tf.keras.callbacks.LearningRateScheduler(schedule=lambda epoch: 0.0 if epoch == 0 else 1e-2),
                    tf.keras.callbacks.EarlyStopping(monitor="loss", patience=99, restore_best_weights=True),
                    tf.keras.callbacks.ReduceLROnPlateau(monitor="loss", factor=0.5, patience=10),
                    EpochProgress(),
                ],
            )

            # Work out how much the loss has improved by
            original_loss = history.history["loss"][0]
            best_loss = min(history.history["loss"])
            print("Loss improved by {:.2g}%".format(100.0 * (original_loss - best_loss) / original_loss))
            print()

            # Update the intrinsics in the config
            config["lens"]["focal_length"] = float(model.focal_length.numpy())
            config["lens"]["centre"][0] = float(model.centre[0].numpy())
            config["lens"]["centre"][1] = float(model.centre[1].numpy())

    # Write out the new model configuration
    for name, config in configurations.items():
        with open(os.path.join(config_path, "{}.yaml".format(name)), "w") as f:
            yaml.dump(config["config"], f)

    # Create the extrinsics dataset
    if not no_extrinsics:

        # Extract the timestamps for each so we can work out which is the best baseline camera
        timestamps = {name: [(*v["timestamp"], v["centres"] is not None) for v in data] for name, data in grids.items()}

        # Create the values for each of the elements
        extrinsic_data = {(a, b): list() for a in grids for b in grids if a != b}
        for n_1, d_1 in grids.items():
            for v_1 in d_1:
                if v_1["centres"] is not None:
                    for n_2, d_2 in grids.items():
                        if n_1 != n_2:
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
                                        (v_1["centres"], v_2a["centres"] * (1.0 - factor) + v_2b["centres"] * factor)
                                    )

        # Put all the values into tensors
        extrinsic_data = {
            k: (
                tf.reshape(np.array([v[0] for v in data], dtype=np.float32), (len(data), COLS, ROWS, 2)),
                tf.reshape(np.array([v[1] for v in data], dtype=np.float32), (len(data), COLS, ROWS, 2)),
            )
            for k, data in extrinsic_data.items()
        }

        # Calculate the per point weight so that the more unique the point is the better its weight is (per camera)
        # Do this before we predict using the model so this is still in normalized pixel space
        sample_weights = []
        for v in extrinsic_data.values():

            # Calculate the distance of this pair of grids from all the other pairs of grids
            block = tf.concat(v, axis=-1)
            position_distance = tf.stack(
                [tf.reduce_mean(tf.sqrt(tf.reduce_sum(tf.math.squared_difference(block, p)))) for p in block], axis=0,
            )
            position_distance = tf.multiply(
                tf.divide(position_distance, tf.reduce_sum(position_distance)), block.shape[0]
            )
            sample_weights.append(position_distance)
        sample_weights = tf.concat(sample_weights, axis=-1)

        # Apply the intrinsic parameters to the pixel coordinates to get unit vectors
        extrinsic_data = {
            k: (configurations[k[0]]["model"].predict(v[0]), configurations[k[1]]["model"].predict(v[1]))
            for k, v in extrinsic_data.items()
        }

        # Calculate the class weighting so each camera pair has equal weight in the sampling
        n_samples = sum([v[0].shape[0] for v in extrinsic_data.values()])
        class_weights = tf.concat(
            [
                np.full(
                    shape=v[0].shape[0],
                    fill_value=(n_samples / (v[0].shape[0] * len(extrinsic_data))),
                    dtype=np.float32,
                )
                for v in extrinsic_data.values()
            ],
            axis=-1,
        )

        # Weights are the combination of the two
        weights = tf.multiply(sample_weights, class_weights)

        # Grab all the Hpcs and make mappings from ids to names
        Hpcs = [
            (k, np.array(configurations[k]["config"]["lens"]["Hpc"], dtype=np.float32)) for k in sorted(grids.keys())
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

        model = ExtrinsicCluster(Hpcs)

        model.compile(
            run_eagerly=True,
            optimizer=tf.keras.optimizers.Adam(learning_rate=1e-2),
            loss=ExtrinsicLoss(ROWS, COLS, 0.04),
            metrics=[alignment, GridError(ROWS, COLS, 0.04)],
        )

        history = model.fit(
            x=[i_0, x_0, i_1, x_1],
            y=tf.zeros(x_0.shape[0], dtype=tf.float32),
            epochs=1000000,
            batch_size=x_0.shape[0],
            sample_weight=weights,
            shuffle=True,
            callbacks=[
                # 0 Learning rate for the first epoch so that if our current optimisation is the best we don't lose it
                tf.keras.callbacks.LearningRateScheduler(schedule=lambda epoch: 0.0 if epoch == 0 else 1e-2),
                tf.keras.callbacks.EarlyStopping(monitor="loss", patience=99, restore_best_weights=True),
                tf.keras.callbacks.ReduceLROnPlateau(monitor="loss", factor=0.5, patience=10),
            ],
        )

        for i, v in enumerate(model.translations):
            print("{}->{}: {}".format(name_for_id[0], name_for_id[i], v))

        import pdb

        pdb.set_trace()
        # TODO convert the extrinsic_data into a_i,a,b_i,b pairs
        # TODO run each of the values through the intrinsic model
        # TODO create the model that will apply the line equation conversion for the points
        # TODO the distance between two of the cameras is fixed to 1 unit, ideally these two are the furthest away since it makes it easiest to correct the scale later
        # TODO we should read in the current orientation values from the configuration files to initialise the system

        # Convert into a numpy array
