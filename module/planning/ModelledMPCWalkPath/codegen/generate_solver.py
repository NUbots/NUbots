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
"""Generate ModelledMPCWalkPath's acados solver (the C code in ../solver).

The problem is MPC-2: MPCWalkPath's walk MPC (MPC-1), planning with an identified Hammerstein model of how the walk
policy responds to its velocity command instead of assuming the command is delivered. Every 10 Hz tick it solves, in
the robot frame at the time of the solve, over N steps of dt:

    state   s = (x, y, θ, z, u_prev)      pose, the model's 9 lag states, and the command applied in the previous step
    input   a = (ax, ay, aω)              the change of command per second
    step    u = u_prev + a·dt             command held during the step, then dt/Ts model steps of
                v = Σⱼ zᵢⱼ                                     delivered velocity (vx, vy, ω), per output axis i
                pose⁺ = pose + Ts·(R(θ + ω·Ts/2)·(vx, vy), ω)
                zᵢⱼ⁺ = pᵢⱼ·zᵢⱼ + bᵢⱼ·fᵢⱼ(uⱼ)                  the Hammerstein model, one path per (output i, command j)

    minimise  Σₖ w_pos·ρ(‖pₖ − p*‖) + w_head·hₖ(θₖ) + Σᵢ w_eff,i·vₖ,ᵢ² + Σᵢ w_rate,i·(aₖ,ᵢ·dt)² + slack penalty

    subject to  velocity limits on u_prev at stages 1..N, acceleration limits on a,
                nᵢ·u_prev ≤ dᵢ at stages 1..N, the capability envelope (soft, L1 + L2 penalty on the slack)
                ‖pₖ − oⱼ‖² ≥ r² for each active obstacle j (soft, L1 + L2 penalty on the slack)

The costs are MPC-1's (see MPCWalkPath's generator), except that effort is on the delivered velocity v rather than the
command, as the command's size says little about what the robot does with it.

The model is MATLAB's idnlhw with a piecewise-linear input nonlinearity per path and nb = nf = nk = 1:

    fᵢⱼ(u) = Lᵢⱼ·u + dᵢⱼ + Σₙ cᵢⱼₙ·|u + tᵢⱼₙ|        (breakpoints at −tᵢⱼₙ)

Its kinks are smoothed here, |x| ≈ √(x² + εⱼ²) with a width εⱼ per command axis, so the solver's exact Hessian is
defined everywhere. A wider ε also merges breakpoints that are close together, which removes narrow non-monotonic
notches a fit can leave (a larger command delivering less), as those are local minima the solver gets stuck in.

The model's parameters are per-stage parameters (written into the module's configuration by import_model.py), so a
refitted model of the same structure needs no regeneration. Its structure is fixed here:
MODEL_TS, UNITS breakpoints per path, first-order paths with a one-step delay.

The capability envelope is ENVELOPE_ROWS linear constraints on the command (written into the module's configuration
by import_envelope.py); their coefficients are set at runtime, and unused rows are switched off.

Only N, dt, the model's structure, the number of envelope rows and the number of obstacle slots are fixed by the
generated code. Run with a Python
environment that has casadi and acados_template, from the same acados release the Docker image builds (see
ACADOS_VERSION below), e.g. walk-mpc's:

    uv run --project ~/walk-mpc python module/planning/ModelledMPCWalkPath/codegen/generate_solver.py

The generated C is committed, so the build doesn't need Python, CasADi or acados_template.
"""

import argparse
import shutil
import tempfile
from pathlib import Path

import casadi as ca
import numpy as np

ACADOS_VERSION = "v0.6.0"
NAME = "modelled_mpc_walk_path"

HORIZON = 25  # steps
DT = 0.1  # s, also the planner period
MAX_OBSTACLES = 4  # obstacle slots, unused ones are switched off
ENVELOPE_ROWS = 24  # linear constraints on the command, unused ones are switched off

# The Hammerstein model's structure
MODEL_TS = 0.02  # s, the model's sample time (the policy's control period)
SUBSTEPS = round(DT / MODEL_TS)
UNITS = 6  # breakpoints of each path's piecewise-linear map
PATHS = 9  # one per (output axis, command axis), output-major: vx←(vx, vy, ω), vy←(...), ω←(...)

# One path's parameters, at P_MODEL + PATH_SIZE·path
PATH_GAIN = 0  # b, the linear block's numerator
PATH_POLE = 1  # p, the linear block's pole
PATH_LINEAR = 2  # L
PATH_OFFSET = 3  # d
PATH_COEF = 4  # c (UNITS)
PATH_TRANSLATION = PATH_COEF + UNITS  # t (UNITS)
PATH_SIZE = PATH_TRANSLATION + UNITS

# Parameter vector layout, per stage. Mirrored into the generated modelled_mpc_walk_path_layout.h.
P_TARGET = 0  # target x, y, θ in the robot frame (3)
P_GATE = 3  # heading fade-in gₖ
P_BEARING = 4  # bearing βₖ from the predicted position to the target
P_W_POSITION = 5
P_W_HEADING = 6
P_W_FACE = 7
P_HUBER_DELTA = 8
P_W_EFFORT = 9  # per axis (3)
P_W_RATE = 12  # per axis (3)
P_OBSTACLE_RADIUS = 15
P_OBSTACLES = 16  # x, y of each slot (2 · MAX_OBSTACLES)
P_ACTIVE = P_OBSTACLES + 2 * MAX_OBSTACLES  # 1 for a real obstacle, 0 for an unused slot (MAX_OBSTACLES)
P_KINK_SMOOTHING = P_ACTIVE + MAX_OBSTACLES  # ε of |x| ≈ √(x² + ε²) for the paths of each command axis (3)
P_MODEL = P_KINK_SMOOTHING + 3  # the model's paths (PATHS · PATH_SIZE)
NP = P_MODEL + PATHS * PATH_SIZE

# State layout
S_POSE = 0  # x, y, θ (3)
S_LAG = 3  # the model's lag states z, one per path (PATHS)
S_COMMAND = S_LAG + PATHS  # the command applied in the previous step (3)
NX = S_COMMAND + 3


def path_input(q, u, eps):
    """The path's piecewise-linear map f(u), q being its PATH_SIZE parameters, with its kinks smoothed over eps"""
    coef, translation = q[PATH_COEF : PATH_COEF + UNITS], q[PATH_TRANSLATION : PATH_TRANSLATION + UNITS]
    return q[PATH_LINEAR] * u + q[PATH_OFFSET] + ca.dot(coef, ca.sqrt((u + translation) ** 2 + eps**2))


def step(pose, z, u, model, eps):
    """One model step: the pose moves with the delivered velocity, then the lags take the command"""
    v = ca.vertcat(*[ca.sum1(z[3 * i : 3 * i + 3]) for i in range(3)])
    heading = pose[2] + v[2] * MODEL_TS / 2
    c, s = ca.cos(heading), ca.sin(heading)
    pose = pose + MODEL_TS * ca.vertcat(c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])
    z = ca.vertcat(
        *[
            model[n, PATH_POLE] * z[n] + model[n, PATH_GAIN] * path_input(model[n, :].T, u[n % 3], eps[n % 3])
            for n in range(PATHS)
        ]
    )
    return pose, z


def delivered(z):
    return ca.vertcat(*[ca.sum1(z[3 * i : 3 * i + 3]) for i in range(3)])


def build_ocp():
    from acados_template import AcadosModel, AcadosOcp

    m = MAX_OBSTACLES
    s = ca.SX.sym("s", NX)
    a = ca.SX.sym("a", 3)
    p = ca.SX.sym("p", NP)

    target = p[P_TARGET : P_TARGET + 3]
    gate, bearing = p[P_GATE], p[P_BEARING]
    w_position, w_heading, w_face, delta = p[P_W_POSITION], p[P_W_HEADING], p[P_W_FACE], p[P_HUBER_DELTA]
    w_effort, w_rate = p[P_W_EFFORT : P_W_EFFORT + 3], p[P_W_RATE : P_W_RATE + 3]
    radius = p[P_OBSTACLE_RADIUS]
    obstacles = ca.reshape(p[P_OBSTACLES : P_OBSTACLES + 2 * m], 2, m)
    active = p[P_ACTIVE : P_ACTIVE + m]
    eps = p[P_KINK_SMOOTHING : P_KINK_SMOOTHING + 3]
    model_params = ca.reshape(p[P_MODEL:NP], PATH_SIZE, PATHS).T  # one row per path

    pose, z = s[S_POSE : S_POSE + 3], s[S_LAG : S_LAG + PATHS]
    u = s[S_COMMAND:] + a * DT
    for _ in range(SUBSTEPS):
        pose, z = step(pose, z, u, model_params, eps)

    model = AcadosModel()
    model.name = NAME
    model.x, model.u, model.p = s, a, p
    model.disc_dyn_expr = ca.vertcat(pose, z, u)

    xy, theta = s[S_POSE : S_POSE + 2], s[S_POSE + 2]
    d2 = ca.sumsqr(xy - target[:2])
    position = delta**2 * (ca.sqrt(1 + d2 / delta**2) - 1)
    # w_head·h(θ), written out so that w_head = 0 doesn't divide by zero
    heading = w_heading * gate * (1 - ca.cos(theta - target[2])) + w_face * (1 - gate) * (1 - ca.cos(theta - bearing))
    effort = ca.dot(w_effort, delivered(s[S_LAG : S_LAG + PATHS]) ** 2)
    rate = ca.dot(w_rate, (a * DT) ** 2)
    # Stage k costs the state s_k (reached by the previous step) and the change of command now. At k = 0 the state is
    # fixed, so its terms are a constant; the state at N is costed by the terminal cost with the same weights.
    model.cost_expr_ext_cost = w_position * position + heading + effort + rate
    model.cost_expr_ext_cost_e = w_position * position + heading + effort

    clearance = ca.vertcat(*[active[j] * (radius**2 - ca.sumsqr(xy - obstacles[:, j])) for j in range(m)])
    model.con_h_expr = clearance
    model.con_h_expr_e = clearance

    ocp = AcadosOcp()
    ocp.name = NAME  # without a name, acados appends a hash of the problem to every generated symbol
    ocp.model = model
    ocp.solver_options.N_horizon = HORIZON
    ocp.solver_options.tf = HORIZON * DT
    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"
    ocp.parameter_values = np.zeros(NP)

    # Placeholder limits: the module sets the real ones from its configuration
    command = np.arange(S_COMMAND, S_COMMAND + 3)
    ocp.constraints.idxbu = np.arange(3)
    ocp.constraints.lbu, ocp.constraints.ubu = -np.ones(3), np.ones(3)
    ocp.constraints.idxbx = command
    ocp.constraints.lbx, ocp.constraints.ubx = -np.ones(3), np.ones(3)
    ocp.constraints.idxbx_e = command
    ocp.constraints.lbx_e, ocp.constraints.ubx_e = -np.ones(3), np.ones(3)
    ocp.constraints.x0 = np.zeros(NX)

    # The capability envelope, C·s ≤ ug on the command part of the state, soft. Placeholder rows: the module sets the
    # real ones from its configuration (and switches them off at stage 0, which is pinned to the current state).
    big = 1e9
    r = ENVELOPE_ROWS
    envelope = np.zeros((r, NX))
    envelope[:, S_COMMAND : S_COMMAND + 3] = np.tile(np.eye(3), (r // 3 + 1, 1))[:r]
    for suffix in ("", "_e"):
        setattr(ocp.constraints, "C" + suffix, envelope)
        setattr(ocp.constraints, "lg" + suffix, -big * np.ones(r))
        setattr(ocp.constraints, "ug" + suffix, big * np.ones(r))
        setattr(ocp.constraints, "idxsg" + suffix, np.arange(r))
    ocp.constraints.D = np.zeros((r, 3))
    # Stage 0 has the envelope's slacks only; stages 1..N have the envelope's, then the obstacles'
    ocp.cost.zl_0, ocp.cost.zu_0 = np.zeros(r), 1000.0 * np.ones(r)  # set from the configuration's w_envelope
    ocp.cost.Zl_0, ocp.cost.Zu_0 = np.zeros(r), np.ones(r)

    for suffix in ("", "_e"):
        setattr(ocp.constraints, "lh" + suffix, -big * np.ones(m))
        setattr(ocp.constraints, "uh" + suffix, np.zeros(m))
        setattr(ocp.constraints, "idxsh" + suffix, np.arange(m))
        setattr(ocp.cost, "zl" + suffix, np.zeros(r + m))
        setattr(ocp.cost, "zu" + suffix, 1000.0 * np.ones(r + m))  # set from w_envelope and w_slack
        setattr(ocp.cost, "Zl" + suffix, np.zeros(r + m))
        setattr(ocp.cost, "Zu" + suffix, np.ones(r + m))

    so = ocp.solver_options
    so.integrator_type = "DISCRETE"
    so.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    so.hessian_approx = "EXACT"
    so.regularize_method = "MIRROR"
    so.levenberg_marquardt = 0.0
    so.globalization = "MERIT_BACKTRACKING"
    so.nlp_solver_type = "SQP"
    so.nlp_solver_max_iter = 10  # set from the configuration's max_iterations
    so.tol = 1e-6
    so.print_level = 0
    return ocp


LAYOUT_HEADER = """\
/* Generated by codegen/generate_solver.py (acados {version}); do not edit. */
#ifndef MODELLED_MPC_WALK_PATH_LAYOUT_H
#define MODELLED_MPC_WALK_PATH_LAYOUT_H

#define MODELLED_MPC_WALK_PATH_DT {dt}
#define MODELLED_MPC_WALK_PATH_MAX_OBSTACLES {m}
#define MODELLED_MPC_WALK_PATH_ENVELOPE_ROWS {rows}

/* The Hammerstein model's structure */
#define MODELLED_MPC_WALK_PATH_MODEL_TS {ts}
#define MODELLED_MPC_WALK_PATH_SUBSTEPS {substeps}
#define MODELLED_MPC_WALK_PATH_UNITS {units}
#define MODELLED_MPC_WALK_PATH_PATHS {paths}

/* One path's parameters, at P_MODEL + PATH_SIZE * path */
{path}

/* Per-stage parameter vector layout */
{indices}

/* State layout */
{state}

#endif /* MODELLED_MPC_WALK_PATH_LAYOUT_H */
"""


def defines(prefix):
    return "\n".join(f"#define MODELLED_MPC_WALK_PATH_{k} {v}" for k, v in globals().items() if k.startswith(prefix))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "solver")
    args = parser.parse_args()

    from acados_template import AcadosOcpSolver

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        ocp = build_ocp()
        ocp.code_gen_options.code_export_directory = str(tmp / "c_generated_code")
        AcadosOcpSolver.generate(ocp, verbose=False)
        generated = tmp / "c_generated_code"

        # Keep the solver and the CasADi functions it calls; drop the example main, the build files and the simulator
        if args.output.exists():
            shutil.rmtree(args.output)
        args.output.mkdir(parents=True)
        for path in sorted(generated.rglob("*")):
            rel = path.relative_to(generated)
            if path.is_dir() or path.suffix not in (".c", ".h") or rel.name.startswith("main_"):
                continue
            if rel.parent == Path(".") and not rel.name.startswith("acados_solver_"):
                continue
            (args.output / rel).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy(path, args.output / rel)

    # NP and NX are in the solver header
    (args.output / f"{NAME}_layout.h").write_text(
        LAYOUT_HEADER.format(
            version=ACADOS_VERSION,
            dt=DT,
            m=MAX_OBSTACLES,
            rows=ENVELOPE_ROWS,
            ts=MODEL_TS,
            substeps=SUBSTEPS,
            units=UNITS,
            paths=PATHS,
            path=defines("PATH_"),
            indices=defines("P_"),
            state=defines("S_"),
        )
    )
    print(f"Wrote {sum(1 for _ in args.output.rglob('*.[ch]'))} files to {args.output}")


if __name__ == "__main__":
    main()
