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
"""Import an identified Hammerstein model of the walk policy into ModelledMPCWalkPath's configuration.

Reads the .mat that walkmodel-identification's fitting saves (a cell array `models` of three MISO idnlhw objects, one
per output vx, vy, wz, plus Ts and gateThr) without MATLAB, using the mat-io package, which decodes MATLAB's class
objects. It checks the model has the structure the generated solver assumes, then rewrites the `model:` section at the
end of data/config/ModelledMPCWalkPath.yaml:

    uv run --no-project --with mat-io python module/planning/ModelledMPCWalkPath/codegen/import_model.py \\
        ~/walkmodel-identification/hammerstein_no_gate_model_k1.mat --policy poi732yy

Each path is idnlhw's input nonlinearity for one command axis (an idPiecewiseLinear) and the linear block from it to
the output, B/F with nb = nf = nk = 1. idPiecewiseLinear stores its map as

    f(u) = LinearCoef·u + OutputOffset + Σₙ OutputCoef[n]·|u + Translation[n]|

(breakpoints at −Translation). That form isn't documented: it was the one of the candidates whose steady state,
Σⱼ b·fᵢⱼ(uⱼ)/(1 − p), matched the poi732yy policy's measured steady-state velocity grid (walk-mpc's
data/k1_poi732yy_response.json; RMSE 0.04 m/s, 0.07 m/s and 0.12 rad/s inside |command| ≤ 1, against 0.1–6 for the
others).
"""

import argparse
import hashlib
from pathlib import Path

import numpy as np

# The structure codegen/generate_solver.py fixes
MODEL_TS = 0.02
UNITS = 6
AXES = ("vx", "vy", "wz")
MARKER = "# ---- The walk policy's response model. Written by codegen/import_model.py: edit by re-importing. ----"


def scalar(x):
    return float(np.ravel(x)[0])


def read(mat_path):
    import matio

    data = matio.load_from_mat(str(mat_path))
    ts, gate = scalar(data["Ts"]), scalar(data["gateThr"])
    if abs(ts - MODEL_TS) > 1e-12:
        raise SystemExit(f"Ts is {ts}, but the solver is generated for {MODEL_TS}: regenerate it with MODEL_TS = {ts}")
    if gate != 0:
        raise SystemExit(f"gateThr is {gate}: the solver can't model a standing gate (it isn't differentiable)")

    models = list(np.ravel(data["models"]))
    if len(models) != 3:
        raise SystemExit(f"expected 3 models (vx, vy, wz), got {len(models)}")
    paths = {}
    for i, model in enumerate(models):
        if model.classname != "idnlhw":
            raise SystemExit(f"model {i} is a {model.classname}, not an idnlhw")
        p = model.properties
        output = str(np.ravel(p["OutputName_"])[0])
        inputs = [str(x) for x in np.ravel(p["InputName_"])]
        if output != AXES[i] or inputs != ["cmd_" + a for a in AXES]:
            raise SystemExit(f"model {i} maps {inputs} to {output}, expected cmd_vx, cmd_vy, cmd_wz to {AXES[i]}")
        if p["OutputNonlinearity_"].classname != "idUnitGain":
            raise SystemExit(f"{output} has an output nonlinearity ({p['OutputNonlinearity_'].classname})")
        if abs(scalar(p["Ts_"]) - ts) > 1e-12:
            raise SystemExit(f"{output}'s sample time {scalar(p['Ts_'])} isn't Ts")
        for name in ("prvFcnInputScale", "prvOutputScale"):
            if not np.allclose(p[name], 1):
                raise SystemExit(f"{output} has a {name} other than 1, which this import doesn't apply")
        for name in ("prvFcnInputCenter", "prvOutputCenter"):
            if not np.allclose(p[name], 0):
                raise SystemExit(f"{output} has a {name} other than 0, which this import doesn't apply")
        if not np.all(np.ravel(p["nk_"]) == 1):
            raise SystemExit(f"{output}'s delays are {np.ravel(p['nk_'])}, expected 1")

        for j, command in enumerate(AXES):
            nl = np.ravel(p["InputNonlinearity_"])[j]
            if nl.classname != "idPiecewiseLinear":
                raise SystemExit(f"{output}←{command} is a {nl.classname}, not an idPiecewiseLinear")
            par = nl.properties["Parameters_"]
            field = lambda f: np.ravel(np.ravel(par[f])[0]).astype(float)
            coef, translation = field("OutputCoef"), field("Translation")
            if len(coef) != UNITS or len(translation) != UNITS:
                raise SystemExit(f"{output}←{command} has {len(coef)} breakpoints, the solver is generated for {UNITS}")
            b = np.ravel(np.ravel(p["Btail"])[j]).astype(float)
            f = np.ravel(np.ravel(p["f_"])[j]).astype(float)
            if len(b) != 1 or len(f) != 2 or f[0] != 1:
                raise SystemExit(f"{output}←{command} isn't first order (B = {b}, F = {f})")
            pole = -f[1]
            if not 0 <= abs(pole) < 1:
                raise SystemExit(f"{output}←{command} is unstable (pole {pole})")
            paths[(output, command)] = {
                "gain": b[0],
                "pole": pole,
                "linear": field("LinearCoef")[0],
                "offset": field("OutputOffset")[0],
                "coef": coef,
                "translation": translation,
            }
    # The command amplitude the identification runs spanned, per axis: beyond it the maps are extrapolated
    identified_range = np.ravel(data["umax"]).astype(float) if "umax" in data else None
    return ts, paths, identified_range


def number(x):
    return repr(float(x))


def to_yaml(ts, paths, identified_range, source, policy):
    lines = [
        MARKER,
        "# The Hammerstein model of how the walk policy's gait-averaged velocity responds to its command, which the",
        "# MPC plans with. One path per (output, command); each output is the sum of its three paths' lag states z:",
        "#   z⁺ = pole·z + gain·f(command) every sample_time",
        "#   f(u) = linear·u + offset + Σₙ coef[n]·|u + translation[n]|",
        "model:",
        f"  source: {source}",
    ]
    if policy:
        lines.append(f"  policy: {policy}")
    lines.append(f"  sample_time: {number(ts)}")
    if identified_range is not None:
        lines.append(
            "  # The command amplitude per axis [vx, vy, wz] the identification spanned: the maps extrapolate beyond"
        )
        lines.append(f"  identified_range: [{', '.join(number(v) for v in identified_range)}]")
    lines.append("  paths:")
    for output in AXES:
        lines.append(f"    {output}:")
        for command in AXES:
            q = paths[(output, command)]
            lines.append(f"      {command}:")
            for key in ("gain", "pole", "linear", "offset"):
                lines.append(f"        {key}: {number(q[key])}")
            for key in ("coef", "translation"):
                # Block lists: at full precision, six numbers don't fit on a line
                lines.append(f"        {key}:")
                lines.extend(f"          - {number(v)}" for v in q[key])
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mat", type=Path, help="the .mat saved by the fit")
    parser.add_argument("--policy", help="the walk policy the model was identified on, recorded with the model")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "data" / "config" / "ModelledMPCWalkPath.yaml",
    )
    args = parser.parse_args()

    ts, paths, identified_range = read(args.mat)
    digest = hashlib.sha256(args.mat.read_bytes()).hexdigest()[:12]
    block = to_yaml(ts, paths, identified_range, f"{args.mat.name} (sha256 {digest}…)", args.policy)

    text = args.config.read_text()
    head = text.split(MARKER)[0].rstrip("\n")
    args.config.write_text(head + "\n\n" + block)
    for (output, command), q in paths.items():
        tau = -ts / np.log(abs(q["pole"])) if q["pole"] != 0 else 0.0
        print(f"{output}←{command}: pole {q['pole']:+.4f} (τ {tau:.2f} s)")
    print(f"Wrote the model to {args.config}")


if __name__ == "__main__":
    main()
