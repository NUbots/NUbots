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
"""Fit ModelledMPCWalkPath's capability envelope to a walk policy evaluation and write it into the configuration.

Reads mjlab's evaluation of the policy (scripts/eval in mjlab: the three two-axis command grids, grid_vx_vy_*,
grid_vx_wz_* and grid_vy_wz_*, each command held for a minute), and keeps the commands the policy both survived and
tracked: steady-state planar velocity error at most --tolerance m/s and yaw rate error at most --tolerance rad/s. In
each plane it fits the polygon with --edges fixed edge directions that just contains those commands, and writes its
edges as linear constraints on the command, n·u ≤ d, which the MPC holds softly at every step:

    uv run --no-project python module/planning/ModelledMPCWalkPath/codegen/import_envelope.py \\
        ~/mjlab/logs/eval/k1-poi732yy-evaluation

The polygons are taken over the kept commands, so the cells inside them that the policy tracks badly near zero (its
dead zones, and turning on the spot) stay in: the model accounts for those. The velocity limits (max_velocity,
max_backward_velocity) become the envelope's extent along each axis, clipped to the range of commands the response
model was identified on (the model's identified_range), as the model is extrapolated beyond it, and made symmetric on
vy and ω, as the limits are.

Each grid holds the third axis fixed (the vy × ω grid at the vx it was run with), so the envelope is the intersection
of three cylinders: combinations of all three axes weren't measured.
"""

import argparse
import csv
import re
import textwrap
from pathlib import Path

import numpy as np

AXES = ("vx", "vy", "wz")
PLANES = (("vx", "vy"), ("vx", "wz"), ("vy", "wz"))
ROWS = 24  # the generated solver's envelope rows (generate_solver.py's ENVELOPE_ROWS)
MARKER = "# ---- The capability envelope. Written by codegen/import_envelope.py: edit by re-importing. ----"
MODEL_MARKER = "# ---- The walk policy's response model."


def grid(evaluation, a, b):
    """The grid over (a, b): its commands, which of them were kept, and the value of the fixed axis"""
    paths = sorted(evaluation.glob(f"grid_{a}_{b}_*/per_env.csv"))
    if len(paths) != 1:
        raise SystemExit(f"expected one grid_{a}_{b}_* in {evaluation}, found {len(paths)}")
    rows = list(csv.DictReader(open(paths[0])))
    command = np.array([[float(r[f"command_{x}"]) for x in AXES] for r in rows])
    error = np.array([[float(r[f"error_{x}"]) for x in AXES] for r in rows])
    survived = np.array([float(r["survived"]) > 0.5 for r in rows])
    fixed = [x for x in AXES if x not in (a, b)][0]
    return command, error, survived, float(np.median(command[:, AXES.index(fixed)])), fixed, paths[0].parent.name


def fit(evaluation, tolerance, edges):
    rows, comments, extent = [], [], {x: [np.inf, np.inf] for x in AXES}  # extent: [backward, forward] per axis
    for a, b in PLANES:
        command, error, survived, fixed_value, fixed, name = grid(evaluation, a, b)
        kept = survived & (np.hypot(error[:, 0], error[:, 1]) <= tolerance) & (np.abs(error[:, 2]) <= tolerance)
        i, j = AXES.index(a), AXES.index(b)
        points = command[kept][:, [i, j]]
        angles = np.arange(edges) * 2 * np.pi / edges
        normals = np.round(np.stack([np.cos(angles), np.sin(angles)], 1), 12)
        bounds = (points @ normals.T).max(0)
        inside = np.all(command[:, [i, j]] @ normals.T <= bounds + 1e-9, 1)
        comments.append(
            f"{a} × {b} ({name}, {fixed} = {fixed_value:g}): {kept.sum()} of {len(kept)} commands kept, "
            f"{np.sum(inside & ~kept)} of the {inside.sum()} inside the polygon not"
        )
        for n, d in zip(normals, bounds):
            row = np.zeros(4)
            row[i], row[j], row[3] = n[0], n[1], d
            rows.append(row)
        for k, x in ((i, a), (j, b)):
            extent[x][0] = min(extent[x][0], -points[:, 0 if k == i else 1].min())
            extent[x][1] = min(extent[x][1], points[:, 0 if k == i else 1].max())
    if len(rows) > ROWS:
        raise SystemExit(f"{len(rows)} envelope rows, but the solver is generated for {ROWS}: use fewer --edges")
    return rows, comments, extent


def number(x):
    return "0" if x == 0 else f"{x:.6g}"  # without a "-0"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("evaluation", type=Path, help="the evaluation directory, holding the grid_*_* runs")
    parser.add_argument("--tolerance", type=float, default=0.2, help="steady-state error kept (m/s and rad/s)")
    parser.add_argument("--edges", type=int, default=8, help="polygon edges per plane")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "data" / "config" / "ModelledMPCWalkPath.yaml",
    )
    args = parser.parse_args()

    rows, comments, extent = fit(args.evaluation, args.tolerance, args.edges)
    text = args.config.read_text()
    match = re.search(r"identified_range: \[([^\]]*)\]", text)
    identified = np.array([float(v) for v in match.group(1).split(",")]) if match else np.full(3, np.inf)

    forward = [min(extent["vx"][1], identified[0])]
    forward += [min(*extent[x], identified[k]) for k, x in ((1, "vy"), (2, "wz"))]
    backward = min(extent["vx"][0], identified[0])
    text = re.sub(r"(?m)^max_velocity: .*$", f"max_velocity: [{', '.join(number(v) for v in forward)}]", text)
    text = re.sub(r"(?m)^max_backward_velocity: .*$", f"max_backward_velocity: {number(backward)}", text)

    note = (
        f"Fitted to {args.evaluation.name}: the commands the policy survived and tracked to within "
        f"{args.tolerance:g} m/s and rad/s in steady state, as one polygon per command plane. Each row "
        "[n_vx, n_vy, n_wz, d] is the constraint n_vx·vx + n_vy·vy + n_wz·wz ≤ d on the command, held softly "
        "(w_envelope)."
    )
    block = [MARKER] + ["# " + line for line in textwrap.wrap(note, 116)]
    block += [f"#   {c}" for c in comments]
    block += ["envelope:"]
    for k, row in enumerate(rows):
        if k % args.edges == 0:
            block.append(f"  # {PLANES[k // args.edges][0]} × {PLANES[k // args.edges][1]}")
        block.append(f"  - [{', '.join(number(v) for v in row)}]")

    # The envelope sits between the configuration and the model
    head, model = text.split(MODEL_MARKER, 1)
    head = head.split(MARKER)[0].rstrip("\n")
    args.config.write_text(head + "\n\n" + "\n".join(block) + "\n\n" + MODEL_MARKER + model)
    print("\n".join(comments))
    print(f"max_velocity {np.round(forward, 3)}, max_backward_velocity {backward:.3f}")
    print(f"Wrote {len(rows)} envelope rows to {args.config}")


if __name__ == "__main__":
    main()
