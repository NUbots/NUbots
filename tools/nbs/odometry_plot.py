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
"""
Plot the K1's commanded, mocap and odometry body velocities (x, y, yaw) from a data/odometry_mocap
recording:

    ./b nbs odometry_plot recordings/odometry_mocap/<recording>.nbs [--output plot.png]

  - command:  WalkState.velocity_target, what skill::K1Walk asked the controller for;
  - mocap:    RobotPoseGroundTruth.Hft differentiated over --lag seconds either side, rotated into the
              robot's body frame;
  - odometry: BoosterOdometryTwist, the controller's own body velocity.

The mocap arrives in bursts, so its poses are timed by the NatNet capture timestamp of the MotionCapture
frame each came from rather than when it was received. The remaining time offset and the mocap rigid
body's heading offset from the robot are fitted as in odometry_fit, unless given.
"""

import math

import numpy as np

from utility.nbs import LinearDecoder

from .odometry_fit import _planar_from_iso3, fit_time_offset


def register(command):
    command.description = "Plot commanded, mocap and odometry velocities from an odometry_mocap recording"
    command.add_argument("files", metavar="files", nargs="+", help="The nbs recordings to plot")
    command.add_argument("--lag", type=float, default=0.05, help="Half width (s) of the mocap central difference")
    command.add_argument("--max-gap", type=float, default=0.1, help="Mocap gap (s) beyond which no velocity is given")
    command.add_argument("--time-offset", type=float, default=None, help="Mocap time offset (s); default: fitted")
    command.add_argument("--heading-offset", type=float, default=None, help="Mocap body heading (deg); default: fitted")
    command.add_argument("--output", help="Save the figure here instead of showing it")


def load(files):
    """Series as (t, x, y, yaw) or (t, vx, vy, wz) arrays: odometry poses, mocap poses (timed by capture),
    commands and odometry twists; and the times of mode changes (which reset the odometry)."""
    odometry, truth, command, twist, modes = [], [], [], [], []
    captured = {}  # Rigid body position (as Mocap.cpp puts it in the field) -> (received, capture time)
    types = [
        "message.booster.BoosterOdometry",
        "message.booster.BoosterOdometryTwist",
        "message.booster.BoosterModeState",
        "message.behaviour.state.WalkState",
        "message.localisation.RobotPoseGroundTruth",
        "message.input.MotionCapture",
    ]
    for packet in LinearDecoder(*files, types=types, show_progress=True):
        t, name, msg = packet.emit_timestamp * 1e-6, packet.type.name, packet.msg
        if name == "message.booster.BoosterOdometry":
            odometry.append((t, msg.x, msg.y, msg.theta))
        elif name == "message.booster.BoosterOdometryTwist":
            twist.append((t, msg.linear.x, msg.linear.y, msg.angular.z))
        elif name == "message.booster.BoosterModeState":
            modes.append((t, int(msg.mode)))
        elif name == "message.behaviour.state.WalkState":
            command.append((t, msg.velocity_target.x, msg.velocity_target.y, msg.velocity_target.z))
        elif name == "message.localisation.RobotPoseGroundTruth":
            truth.append(_planar_from_iso3(msg.Hft))
        else:
            for body in msg.rigid_bodies:
                captured[(np.float32(-body.position.y), np.float32(body.position.x))] = (t, msg.natnet_timestamp)

    # Time each pose by its frame's capture, on the robot's clock by the least delayed frame
    frames = np.array([captured.get((np.float32(x), np.float32(y)), (np.nan, np.nan)) for x, y, _ in truth])
    found = ~np.isnan(frames[:, 0])
    if not found.any():
        raise SystemExit("No mocap poses matched a MotionCapture frame")
    print(f"Timed {found.sum()} of {len(truth)} mocap poses by their capture")
    clock = np.min(frames[found, 0] - frames[found, 1])
    truth = np.column_stack([frames[found, 1] + clock, np.array(truth)[found]])
    truth = truth[np.argsort(truth[:, 0])]

    def array(rows):
        return np.array(sorted(rows)).reshape(-1, 4)

    modes.sort()
    resets = np.array([t for (t, mode), (_, previous) in zip(modes[1:], modes) if mode != previous])
    return array(odometry), truth, array(command), array(twist), resets


def mocap_velocity(truth, lag, max_gap, heading_offset):
    """Body frame (vx, vy, wz) of the mocap poses (t, x, y, yaw), by central differences over +-lag."""
    t, pose = truth[:, 0], truth[:, 1:].copy()
    pose[:, 2] = np.unwrap(pose[:, 2])
    before = np.stack([np.interp(t - lag, t, p) for p in pose.T], axis=1)
    after = np.stack([np.interp(t + lag, t, p) for p in pose.T], axis=1)
    v = (after - before) / (2 * lag)

    # Rotate the field velocity into the robot's body frame
    h = pose[:, 2] - heading_offset
    c, s = np.cos(h), np.sin(h)
    v[:, 0], v[:, 1] = c * v[:, 0] + s * v[:, 1], -s * v[:, 0] + c * v[:, 1]

    # No velocity where the difference spans a tracking gap or runs off either end
    gap = np.diff(t) > max_gap
    gaps_before = np.concatenate([[0], np.cumsum(gap)])
    i0, i1 = np.searchsorted(t, t - lag), np.searchsorted(t, t + lag)
    covered = (t - lag >= t[0]) & (t + lag <= t[-1]) & (gaps_before[np.minimum(i1, len(t) - 1)] == gaps_before[i0])
    v[~covered] = np.nan
    return np.column_stack([t, v])


def fit_heading_offset(mocap, twist, max_speed=1.5):
    """Rotation of the mocap body's axes from the robot's that best maps the mocap body's velocity onto
    the odometry twist, in least squares (leaving out the twist's spikes when the odometry resets)."""
    t = mocap[:, 0]
    vb = np.interp(t, twist[:, 0], twist[:, 1]) + 1j * np.interp(t, twist[:, 0], twist[:, 2])
    vm = mocap[:, 1] + 1j * mocap[:, 2]
    ok = ~np.isnan(vm) & (np.abs(vb) < max_speed)
    return float(np.angle(np.sum(np.conj(vm[ok]) * vb[ok])))


def run(files, lag, max_gap, time_offset, heading_offset, output, **kwargs):
    import matplotlib

    if output:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    odometry, truth, command, twist, resets = load(files)

    if time_offset is None:
        time_offset = fit_time_offset(odometry, truth, resets, 0.3)
    truth = truth.copy()
    truth[:, 0] += time_offset
    if np.ptp(truth[:, 0]) == 0:
        print("Warning: the mocap poses all have the same capture time; was the robot tracked?")
    if heading_offset is None:
        heading_offset = fit_heading_offset(mocap_velocity(truth, lag, max_gap, 0.0), twist)
    else:
        heading_offset = math.radians(heading_offset)
    print(f"Mocap time offset {time_offset:+.2f} s, heading offset {math.degrees(heading_offset):+.1f} deg")
    mocap = mocap_velocity(truth, lag, max_gap, heading_offset)

    t0 = min(s[0, 0] for s in (command, twist, mocap) if len(s))
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(14, 9))
    labels = ("x (m/s)", "y (m/s)", "yaw (rad/s)")
    for i, (ax, label) in enumerate(zip(axes, labels)):
        ax.plot(mocap[:, 0] - t0, mocap[:, i + 1], label="mocap", linewidth=1)
        if len(twist):
            ax.plot(twist[:, 0] - t0, twist[:, i + 1], label="odometry", linewidth=1)
        if len(command):
            ax.step(command[:, 0] - t0, command[:, i + 1], where="post", label="command", color="k", linewidth=1.5)
        # Fit the axis to the mocap and command: the odometry spikes when it resets
        bound = np.nanmax(np.abs(np.concatenate([mocap[:, i + 1], command[:, i + 1], [0.05]])))
        ax.set_ylim(-1.3 * bound, 1.3 * bound)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")
    axes[-1].set_xlabel("time (s)")
    fig.suptitle("K1 body velocity: command vs mocap vs odometry")
    fig.tight_layout()

    if output:
        fig.savefig(output, dpi=150)
        print(f"Written to {output}")
    else:
        plt.show()
