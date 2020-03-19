#!/usr/bin/env python3

import json
import os
import re

from google.protobuf.json_format import MessageToJson

from tqdm import tqdm

from .nbs import Decoder


def register(command):
    command.help = "Decode an nbs file and convert it to json"

    # Command arguments
    command.add_argument("files", metavar="files", nargs="+", help="The nbs files to convert to json")
    command.add_argument("--output", "-o", nargs="?", default=os.getcwd(), help="The folder to extract the json data into")

def run(files, output, **kwargs):

    #for packet in Decoder(*files):
    #    out = re.sub(r"\s+", " ", MessageToJson(packet.msg, True))
    #    out = '{{ "type": "{}", "timestamp": {}, "data": {} }}'.format(packet.type, packet.timestamp, out)
    #    # Print as a json object
    #    print(out)

    os.makedirs(output, exist_ok=True)
    decoder = Decoder(*files)

    with tqdm(total=len(decoder), unit="B", unit_scale=True, dynamic_ncols=True) as progress:
        for packet in decoder:

            # Update the progress bar
            progress.n = decoder.bytes_read()
            progress.update(0)

            if packet.type == "message.vision.Balls":
                Hcw = packet.msg.Hcw


                with open(os.path.join(output, "{:012d}.json".format(packet.timestamp)), "w") as f:
                    json.dump(
                        {
                            "timestamp": serializeTimestamp(packet.msg.timestamp),
                            "camera_id": packet.msg.camera_id,
                            "Hcw": [
                                [Hcw.x.x, Hcw.x.y, Hcw.x.z, Hcw.x.t],
                                [Hcw.y.x, Hcw.y.y, Hcw.y.z, Hcw.y.t],
                                [Hcw.z.x, Hcw.z.y, Hcw.z.z, Hcw.z.t],
                                [Hcw.t.x, Hcw.t.y, Hcw.t.z, Hcw.t.t],
                            ],
                            "balls": serializeBalls(packet.msg.balls)
                        },
                        f,
                        indent=4,
                    )


def serializeTimestamp(timestamp):
    return {
        "seconds": timestamp.seconds,
        "nanos": timestamp.nanos,
    }

def serializeBalls(balls):
    ballObjects = []
    for b in balls:
       ballObjects.append(serializeBall(b))
    return ballObjects

def serializeBall(ball):
    return {
        "cone": serializeCone(ball.cone),
        "measurements": serializeMeasurements(ball.measurements),
        "screen_angular": serializefvec2(ball.screen_angular),
        "angular_size": serializefvec2(ball.angular_size),
        "colour": serializefvec4(ball.colour)
    }

def serializeCone(cone):
    return {
        "axis": serializefvec3(cone.axis),
        "gradient": cone.gradient,
        "radius": cone.radius
    }

def serializefvec2(vec):
    return { "x": vec.x, "y": vec.y }

def serializefvec3(vec):
    return { "x": vec.x, "y": vec.y, "z": vec.z }

def serializefvec4(vec):
    return { "x": vec.x, "y": vec.y, "z": vec.z, "t": vec.t }

def serializefmat3(mat):
    return {
        "x": { "x": mat.x.x },
        "y": { "y": mat.y.y },
        "z": { "z": mat.z.z }
    }

def serializeMeasurements(measurements):
    measurementObjects = []
    for m in measurements:
       measurementObjects.append(serializeMeasurement(m))
    return measurementObjects

def serializeMeasurement(measurement):
    return {
        "type": measurement.type,
        "rBCc": serializefvec3(measurement.rBCc),
        "covariance": serializefmat3(measurement.covariance)
    }
