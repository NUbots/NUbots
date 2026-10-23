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

MOCAP = "message.input.MotionCapture"
WALKSTATE = "message.behaviour.state.WalkState"


def register(command):
    command.description = (
        "Extract the mocap position and heading of a walking robot along with its commanded walk velocity into a .mat"
    )
    command.epilog = """
        The heading comes from a three marker set: one marker on each shoulder and one on the front of the torso.
        The markers are projected onto the ground plane and the heading is taken perpendicular to the shoulder line
        (which is insensitive to the body pitching and rolling), with the front marker deciding which way is forward.
    """

    # Command arguments
    command.add_argument(
        "files", metavar="files", nargs="+", help="The nbs files (or directories of nbs files) to extract from"
    )
    command.add_argument("--output", "-o", default="mocap_walk_data.mat", help="The .mat file to write to")
    command.add_argument(
        "--rigid-body",
        default="GaussShoulders",
        help="Name (prefix, case insensitive) of the mocap rigid body and marker set to track",
    )


def _matches(name, prefix):
    return name.lower().startswith(prefix.lower())


def _rigid_body_position(msg, rigid_body):
    for rb in msg.rigid_bodies:
        if _matches(rb.name, rigid_body) and rb.tracking_valid:
            return [rb.position.x, rb.position.y, rb.position.z]
    # The rigid body wasn't tracked in this frame
    return [np.nan] * 3


def _marker_positions(msg, rigid_body):
    for ms in msg.marker_sets:
        if _matches(ms.name, rigid_body) and len(ms.markers) == 3:
            p = np.array([[m.position.x, m.position.y, m.position.z] for m in ms.markers])
            # Occluded markers are reported at the origin
            if not np.any(np.all(p == 0, axis=1)):
                return p
    return np.full((3, 3), np.nan)


def _velocity_target(msg):
    # Snap denormal values to zero and round to the precision the keyboard walk commands are given at
    v = np.array([msg.velocity_target.x, msg.velocity_target.y, msg.velocity_target.z])
    v[np.abs(v) < 1e-32] = 0
    return np.round(v, 3)


def _heading(markers):
    """Unwrapped heading (rad) from the shoulder and front markers projected onto the ground (z up) plane"""
    p = markers[:, :, :2]

    # The shoulders are the two markers furthest apart, the remaining marker is on the front of the torso
    pairs = [(0, 1, 2), (0, 2, 1), (1, 2, 0)]
    dist = np.stack([np.linalg.norm(p[:, a] - p[:, b], axis=-1) for a, b, _ in pairs], axis=-1)
    ok = ~np.isnan(dist).any(axis=-1)
    a, b, f = pairs[np.bincount(np.argmax(dist[ok], axis=-1), minlength=3).argmax()]

    # Forward is perpendicular to the shoulder line, on the side of the front marker
    shoulders = p[:, b] - p[:, a]
    forward = np.stack([-shoulders[:, 1], shoulders[:, 0]], axis=-1)
    front = p[:, f] - 0.5 * (p[:, a] + p[:, b])
    forward *= np.sign(np.sum(forward * front, axis=-1))[:, None]

    yaw = np.arctan2(forward[:, 1], forward[:, 0])
    yaw[ok] = np.unwrap(yaw[ok])
    return yaw


def _capture_time(natnet, received):
    """Map the mocap camera clock onto the nbs clock using the minimum latency envelope"""
    # Lowest receive - capture offset in each 10 second window, which is the capture time plus the minimum latency.
    # A line through these handles any drift between the two clocks
    offset = received - natnet
    window = ((natnet - natnet[0]) // 10).astype(int)
    t, o = [], []
    for w in np.unique(window):
        i = np.flatnonzero(window == w)
        j = i[np.argmin(offset[i])]
        t.append(natnet[j])
        o.append(offset[j])
    fit = np.polyfit(t, o, 1) if len(t) > 1 else [0, o[0]]
    return natnet + np.polyval(fit, natnet)


def run(files, output, rigid_body, **kwargs):
    decoder = LinearDecoder(*resolve_nbs_paths(files), types=[MOCAP, WALKSTATE], show_progress=True)

    # Times are the nbs emit timestamps in microseconds
    mocap_time = []
    natnet_time = []
    frame_number = []
    mocap_position = []
    markers = []
    walkstate_time = []
    target_velocity = []

    for packet in tqdm(decoder, unit="packet", unit_scale=True, dynamic_ncols=True):
        msg = packet.msg
        if packet.type.name == MOCAP:
            mocap_time.append(packet.emit_timestamp)
            natnet_time.append(msg.natnet_timestamp)
            frame_number.append(msg.frame_number)
            mocap_position.append(_rigid_body_position(msg, rigid_body))
            markers.append(_marker_positions(msg, rigid_body))
        else:
            walkstate_time.append(packet.emit_timestamp)
            target_velocity.append(_velocity_target(msg))

    if not mocap_time or not walkstate_time:
        raise RuntimeError("Both MotionCapture and WalkState messages are needed")

    # All times are relative to the first extracted message, in seconds
    first_time = min(mocap_time[0], walkstate_time[0])
    mocap_time = (np.array(mocap_time, dtype=np.float64) - first_time) / 1e6
    walkstate_time = (np.array(walkstate_time, dtype=np.float64) - first_time) / 1e6

    # The receive times arrive in bursts, so use the camera clock for the sample times of the mocap data
    natnet_time = np.array(natnet_time, dtype=np.float64)
    if np.all(np.diff(natnet_time) > 0):
        capture_time = _capture_time(natnet_time, mocap_time)
    else:
        print("Warning: mocap natnet timestamps are missing or not increasing, using the receive times instead")
        capture_time = mocap_time

    markers = np.array(markers)
    if np.all(np.isnan(markers)):
        raise RuntimeError(f"No frames had all 3 markers of the '{rigid_body}' marker set")
    yaw = _heading(markers)
    ok = ~np.isnan(yaw)
    yaw_rate = np.full_like(yaw, np.nan)
    yaw_rate[ok] = np.gradient(yaw[ok], capture_time[ok])

    data = {
        "mocap_time": mocap_time.reshape(-1, 1),
        "mocap_capture_time": capture_time.reshape(-1, 1),
        "frame_number": np.array(frame_number, dtype=np.float64).reshape(-1, 1),
        "mocap_position": np.array(mocap_position, dtype=np.float64).reshape(-1, 3),
        "mocap_yaw": yaw.reshape(-1, 1),
        "mocap_yaw_rate": yaw_rate.reshape(-1, 1),
        "walkstate_time": walkstate_time.reshape(-1, 1),
        "target_velocity": np.array(target_velocity, dtype=np.float32).reshape(-1, 3),
        "first_timestamp_us": np.float64(first_time),
    }

    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    savemat(output, data)

    print(f"Saved {output}")
    for k, v in data.items():
        print(f"  {k:20s} {str(np.shape(v)):14s}")
    print(f"  {np.count_nonzero(~ok)} mocap frames without all 3 markers (yaw is NaN)")
