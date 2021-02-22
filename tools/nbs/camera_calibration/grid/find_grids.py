import copyreg
import math
import multiprocessing
import os
import pickle

from tqdm import tqdm

import cv2
import numpy as np
from utility.nbs import LinearDecoder

from ...images import decode_image, fourcc


# This code here is needed for OpenCV to work with multiprocessing. When finding the grids we use all the cpu cores
# available. However to do this we need to be able to send the OpenCV data back and because of the way that the OpenCV
# module works, pickling doesn't work properly by default as the class is a function. By remapping it here we can fix
# the problem so that it can be pickled properly
def _pickle_keypoints(point):
    return cv2.KeyPoint, (*point.pt, point.size, point.angle, point.response, point.octave, point.class_id)


copyreg.pickle(cv2.KeyPoint().__class__, _pickle_keypoints)


def _packetise_stream(decoder):
    for packet in decoder:
        # Check for compressed images
        if packet.type == "message.output.CompressedImage":
            # Get some useful info into a pickleable format
            yield {
                "camera_name": packet.msg.name,
                "timestamp": (packet.msg.timestamp.seconds, packet.msg.timestamp.nanos),
                "data": packet.msg.data,
                "format": packet.msg.format,
            }


def _process_frame(item, cols, rows):
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
    params.maxArea = math.pi * (img.shape[0] * (1.0 / 22.0)) ** 2

    params.filterByCircularity = False

    params.filterByInertia = False

    detector = cv2.SimpleBlobDetector_create(params)

    # If we failed normally, try with clustering as it often does better near the edges
    # In the documentation cols/rows are specified as cols=points per row, and rows=points per col
    ret, centres = cv2.findCirclesGrid(img, (cols, rows), flags=cv2.CALIB_CB_ASYMMETRIC_GRID, blobDetector=detector)
    if not ret:
        ret, centres = cv2.findCirclesGrid(
            img, (cols, rows), flags=cv2.CALIB_CB_ASYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING, blobDetector=detector
        )

    return {
        "name": item["camera_name"],
        "timestamp": item["timestamp"],
        "image": img,
        "blobs": detector.detect(img),
        "centres": centres if ret else None,
    }


def find_grids(files, cols, rows):

    # Work out what the name of the pickle file will be for caching
    pickle_path = "{}.pickle".format(
        "_".join(os.path.basename(f).replace(".nbs", "").replace(".nbz", "").replace(".gz", "") for f in files)
    )

    # The points from the images that have been gathered
    if not os.path.isfile(pickle_path):

        # Read the nbs file
        with multiprocessing.pool.ThreadPool(multiprocessing.cpu_count()) as pool:
            grids = {}

            print("Detecting asymmetric circles grids")

            results = []

            # Function that updates the results
            def update_results(msg):
                img = msg["image"]
                img = cv2.drawKeypoints(
                    img, msg["blobs"], np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
                )
                if msg["centres"] is not None:
                    img = cv2.drawChessboardCorners(img, (cols, rows), msg["centres"], True)

                # cv2.imshow(msg["name"], img)
                # cv2.waitKey(1)

                # Add this to the grids list
                if msg["name"] not in grids:
                    grids[msg["name"]] = []

                grids[msg["name"]].append(
                    {"timestamp": msg["timestamp"], "centres": msg["centres"], "dimensions": img.shape}
                )

            for msg in _packetise_stream(
                tqdm(LinearDecoder(*files, show_progress=True), unit="packet", unit_scale=True, dynamic_ncols=True)
            ):
                # Add a task to the pool to process
                results.append(pool.apply_async(_process_frame, (msg, cols, rows)))

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
