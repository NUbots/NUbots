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
"""Import InvertedMPCWalkPath's response tables and capability envelope from a walk policy evaluation.

Reads mjlab's evaluation of the policy (scripts/eval in mjlab: the two-axis command grids grid_vx_vy_*, grid_vx_wz_* and
grid_vy_wz_*, each command held for a minute) and writes:

- InvertedMPCWalkPathResponse.yaml, the velocity each command delivered in steady state: the vx × vy grid (ω = 0)
  for translation, and the vx × ω grid (vy = 0) for turning. The module inverts these to turn the velocity the MPC
  plans into the command to send. The whole grid is kept: the velocity limits (max_velocity, max_backward_velocity in
  InvertedMPCWalkPath.yaml) bound the planned velocity, not the command that delivers it, which can be larger.
- The capability envelope at the end of InvertedMPCWalkPath.yaml: in each of the three planes, the polygon with --edges
  fixed edge directions that just contains the velocities the grid's commands delivered, as linear constraints on the
  planned velocity, n·v ≤ d, which the MPC holds softly at every step.

    uv run --no-project --with numpy python module/planning/InvertedMPCWalkPath/codegen/import_response.py \\
        ~/Starman/mjlab_k1/booster_mjlab/logs/eval/k1-poi732yy

Each grid holds the third axis fixed (the vy × ω grid at the vx it was run with), so combinations of all three axes
weren't measured: the envelope is the intersection of three cylinders, and the tables leave out vy's effect on turning
and ω's on translation.
"""

import argparse
import csv
import textwrap
from pathlib import Path

import numpy as np

AXES = ("vx", "vy", "wz")
PLANES = (("vx", "vy"), ("vx", "wz"), ("vy", "wz"))
ROWS = 24  # the solver's envelope rows (Problem::ENVELOPE_ROWS in src/Problem.hpp)
MARKER = "# ---- The capability envelope. Written by codegen/import_response.py: edit by re-importing. ----"
CONFIG = Path(__file__).resolve().parents[1] / "data" / "config"


def grid(evaluation, a, b):
    """The grid over (a, b): its command values along a and b, the commands, what they delivered, and the value of the
    fixed axis"""
    paths = sorted(evaluation.glob(f"grid_{a}_{b}_*/per_env.csv"))
    if len(paths) != 1:
        raise SystemExit(f"expected one grid_{a}_{b}_* in {evaluation}, found {len(paths)}")
    rows = list(csv.DictReader(open(paths[0])))
    command = np.round(np.array([[float(r[f"command_{x}"]) for x in AXES] for r in rows]), 6)
    achieved = np.array([[float(r[f"achieved_{x}"]) for x in AXES] for r in rows])
    if not all(float(r["survived"]) > 0.5 for r in rows):
        raise SystemExit(f"the policy fell in {paths[0].parent.name}: the grid doesn't give a steady state everywhere")
    i, j = AXES.index(a), AXES.index(b)
    along_a, along_b = np.unique(command[:, i]), np.unique(command[:, j])
    if len(command) != len(along_a) * len(along_b):
        raise SystemExit(f"{paths[0].parent.name} isn't a full grid")
    fixed = [x for x in AXES if x not in (a, b)][0]
    return along_a, along_b, command, achieved, float(np.median(command[:, AXES.index(fixed)])), fixed, paths[0].parent


def number(x, digits=4):
    x = round(float(x), digits)
    return "0" if x == 0 else f"{x:g}"  # without a "-0"


def flow(values, digits=4):
    return "[" + ", ".join(number(v, digits) for v in values) + "]"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("evaluation", type=Path, help="the evaluation directory, holding the grid_*_* runs")
    parser.add_argument("--edges", type=int, default=8, help="envelope polygon edges per plane")
    parser.add_argument("--config", type=Path, default=CONFIG / "InvertedMPCWalkPath.yaml")
    parser.add_argument("--response", type=Path, default=CONFIG / "InvertedMPCWalkPathResponse.yaml")
    args = parser.parse_args()

    text = args.config.read_text()

    # The response tables, a row per command
    _, _, translation_command, translation_achieved, _, _, translation = grid(args.evaluation, "vx", "vy")
    _, _, turn_command, turn_achieved, _, _, turn = grid(args.evaluation, "vx", "wz")

    note = (
        f"The velocity each command delivered in steady state, from {args.evaluation.name} ({translation.name} and "
        f"{turn.name}: each command held for a minute in mjlab). "
        "The module inverts it to find the command that delivers the velocity the MPC plans."
    )
    out = ["# The walk policy's measured response. Written by codegen/import_response.py: re-import rather than edit."]
    out += ["# " + line for line in textwrap.wrap(note, 116)]
    out += [
        "",
        "# Translation, with no turn, a row per command on the grid: [vx, vy] commanded, then [vx, vy] delivered (m/s)",
    ]
    out += ["translation:"]
    out += [f"  - {flow([*c[[0, 1]], *v[[0, 1]]])}" for c, v in zip(translation_command, translation_achieved)]
    out += [
        "",
        "# Turning, with no sideways command, a row per command on the grid: [vx (m/s), ω (rad/s)] commanded, then the",
        "# turn rate delivered (rad/s)",
        "turn:",
    ]
    out += [f"  - {flow([*c[[0, 2]], v[2]])}" for c, v in zip(turn_command, turn_achieved)]
    args.response.write_text("\n".join(out) + "\n")

    # The capability envelope, around the velocities delivered in each plane
    rows, comments = [], []
    angles = np.arange(args.edges) * 2 * np.pi / args.edges
    normals = np.round(np.stack([np.cos(angles), np.sin(angles)], 1), 12)
    for a, b in PLANES:
        _, _, command, achieved, fixed_value, fixed, path = grid(args.evaluation, a, b)
        i, j = AXES.index(a), AXES.index(b)
        points = achieved[:, [i, j]]
        bounds = (points @ normals.T).max(0)
        comments.append(f"{a} × {b} ({path.name}, {fixed} commanded {fixed_value:g}): {len(points)} commands")
        for n, d in zip(normals, bounds):
            row = np.zeros(4)
            row[i], row[j], row[3] = n[0], n[1], d
            rows.append(row)
    if len(rows) > ROWS:
        raise SystemExit(f"{len(rows)} envelope rows, but the solver has {ROWS}: use fewer --edges")

    note = (
        f"Fitted to {args.evaluation.name}: the velocities the policy delivered in steady state for the commands "
        "on the grids, as one polygon per plane. Each row [n_vx, n_vy, n_wz, d] is the constraint "
        "n_vx·vx + n_vy·vy + n_wz·wz ≤ d on the planned velocity, held softly (w_envelope)."
    )
    block = [MARKER] + ["# " + line for line in textwrap.wrap(note, 116)]
    block += [f"#   {c}" for c in comments]
    block += ["envelope:"]
    for k, row in enumerate(rows):
        if k % args.edges == 0:
            block.append(f"  # {PLANES[k // args.edges][0]} × {PLANES[k // args.edges][1]}")
        block.append(f"  - {flow(row, 6)}")

    # The envelope is the end of the configuration
    head = text.split(MARKER)[0].rstrip("\n")
    args.config.write_text(head + "\n\n" + "\n".join(block) + "\n")
    print("\n".join(comments))
    print(f"Wrote the response tables to {args.response} and {len(rows)} envelope rows to {args.config}")


if __name__ == "__main__":
    main()
