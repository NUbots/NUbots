#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2026 NUbots
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

import os

import numpy as np
from scipy.io import savemat
from tqdm import tqdm

from utility.nbs import LinearDecoder, resolve_nbs_paths

GROUND_TRUTH = "message.booster.NUSimRobotGroundTruth"
WALKSTATE = "message.behaviour.state.WalkState"


def register(command):
    command.description = (
        "Extract the NUSim ground truth position and heading of a walking robot along with its commanded walk velocity"
        " into a .mat"
    )
    command.epilog = """
        The pose is NUSim's torso (Trunk) ground truth in the simulator world, logged as NUSimRobotGroundTruth (run
        input::NUSimGroundTruth and log that type with DataLogging). The output has the same variables as mocap_walk,
        so the same processing works on either, plus the ground truth torso velocities.
    """

    # Command arguments
    command.add_argument(
        "files", metavar="files", nargs="+", help="The nbs files (or directories of nbs files) to extract from"
    )
    command.add_argument("--output", "-o", default="nusim_walk_data.mat", help="The .mat file to write to")


def _velocity_target(msg):
    # Snap denormal values to zero and round to the precision the keyboard walk commands are given at
    v = np.array([msg.velocity_target.x, msg.velocity_target.y, msg.velocity_target.z])
    v[np.abs(v) < 1e-32] = 0
    return np.round(v, 3)


def _vec3(v):
    return [v.x, v.y, v.z]


def run(files, output, **kwargs):
    decoder = LinearDecoder(*resolve_nbs_paths(files), types=[GROUND_TRUTH, WALKSTATE], show_progress=True)

    # Times are the nbs emit timestamps in microseconds
    gt_time = []
    sim_time = []
    position = []
    heading = []
    velocity = []
    angular_velocity = []
    walkstate_time = []
    target_velocity = []

    for packet in tqdm(decoder, unit="packet", unit_scale=True, dynamic_ncols=True):
        msg = packet.msg
        if packet.type.name == GROUND_TRUTH:
            gt_time.append(packet.emit_timestamp)
            sim_time.append(msg.timestamp.seconds * 1e6 + msg.timestamp.nanos / 1e3)
            # Hst's x, y, z, t are its columns: the torso x axis and position in the simulator world
            H = msg.Hst
            position.append([H.t.x, H.t.y, H.t.z])
            heading.append(np.arctan2(H.x.y, H.x.x))
            velocity.append(_vec3(msg.vTs))
            angular_velocity.append(_vec3(msg.omegaTs))
        else:
            walkstate_time.append(packet.emit_timestamp)
            target_velocity.append(_velocity_target(msg))

    missing = [t for t, v in ((GROUND_TRUTH, gt_time), (WALKSTATE, walkstate_time)) if not v]
    if missing:
        raise RuntimeError(f"The recording has no {' or '.join(missing)} messages")

    # All times are relative to the first extracted message, in seconds
    first_time = min(gt_time[0], walkstate_time[0])
    gt_time = (np.array(gt_time, dtype=np.float64) - first_time) / 1e6
    walkstate_time = (np.array(walkstate_time, dtype=np.float64) - first_time) / 1e6

    # NUSim stamps each snapshot with the wall clock when it was taken, the same clock NUbots logs on when both run on
    # one machine. Use that for the sample times rather than the receive times, which carry the DDS and scheduling delay
    sim_time = (np.array(sim_time, dtype=np.float64) - first_time) / 1e6
    delay = gt_time - sim_time
    if np.all(np.diff(sim_time) > 0) and 0 <= np.median(delay) < 1:
        capture_time = sim_time
    else:
        print("Warning: the ground truth timestamps are not increasing or not on the nbs clock, using the receive times")
        capture_time = gt_time

    yaw = np.unwrap(np.array(heading))
    yaw_rate = np.gradient(yaw, capture_time)

    # The same variables as mocap_walk, so the same processing works on either
    data = {
        "mocap_time": gt_time.reshape(-1, 1),
        "mocap_capture_time": capture_time.reshape(-1, 1),
        "frame_number": np.arange(len(gt_time), dtype=np.float64).reshape(-1, 1),
        "mocap_position": np.array(position, dtype=np.float64).reshape(-1, 3),
        "mocap_yaw": yaw.reshape(-1, 1),
        "mocap_yaw_rate": yaw_rate.reshape(-1, 1),
        "walkstate_time": walkstate_time.reshape(-1, 1),
        "target_velocity": np.array(target_velocity, dtype=np.float32).reshape(-1, 3),
        "first_timestamp_us": np.float64(first_time),
        # Ground truth torso velocities in the simulator world, which mocap doesn't have
        "nusim_velocity": np.array(velocity, dtype=np.float64).reshape(-1, 3),
        "nusim_angular_velocity": np.array(angular_velocity, dtype=np.float64).reshape(-1, 3),
    }

    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    savemat(output, data)

    print(f"Saved {output}")
    for k, v in data.items():
        print(f"  {k:22s} {str(np.shape(v)):14s}")
    print(f"  ground truth delay (receive - snapshot) median {1e3 * np.median(delay):.1f} ms")
