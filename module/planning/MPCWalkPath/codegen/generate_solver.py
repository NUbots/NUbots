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
"""Generate MPCWalkPath's acados solver (the C code in ../solver).

The problem is MPC-1, the kinematic walk MPC prototyped in walk-mpc (walk_mpc/acados_mpc.py). Every 10 Hz tick it
solves, in the robot frame at the time of the solve, over N steps of dt:

    state   s = (x, y, θ, vx, vy, ω)      pose, and the command applied in the previous step
    input   a = (ax, ay, aω)              the change of command per second
    step    u = s[3:] + a·dt              command applied during the step
            pose⁺ = RK4(pose, u, dt),  s⁺ = (pose⁺, u)

    minimise  Σₖ w_pos·ρ(‖pₖ − p*‖) + w_head·hₖ(θₖ) + Σᵢ w_eff,i·uₖ,ᵢ² + Σᵢ w_rate,i·(aₖ,ᵢ·dt)² + slack penalty
              ρ(d) = δ²(√(1 + d²/δ²) − 1)                                    pseudo-Huber distance
              hₖ(θ) = gₖ(1 − cos(θ − θ*)) + (w_face/w_head)(1 − gₖ)(1 − cos(θ − βₖ))   heading, faded in near the target

    subject to  velocity limits on s[3:] at stages 1..N, acceleration limits on a,
                ‖pₖ − oⱼ‖² ≥ r² for each active obstacle j (soft, L1 + L2 penalty on the slack)

gₖ (the heading fade-in) and βₖ (the bearing to the target) are computed by the caller from the initial guess, so
they are parameters. The last pose has no command, so acados's terminal cost costs it with the stage weights.

Only N, dt and the number of obstacle slots are fixed by the generated code. Everything that is tuned lives in the
module's configuration: the cost weights and obstacle radius are per-stage parameters, the velocity and acceleration
limits are bounds, the slack penalty is a cost setting, and the SQP iteration cap is a solver option.

Run with a Python environment that has casadi and acados_template, from the same acados release the Docker image
builds (see ACADOS_VERSION below), e.g. walk-mpc's:

    uv run --project ~/walk-mpc python module/planning/MPCWalkPath/codegen/generate_solver.py

The generated C is committed, so the build doesn't need Python, CasADi or acados_template.
"""

import argparse
import shutil
import tempfile
from pathlib import Path

import casadi as ca
import numpy as np

ACADOS_VERSION = "v0.6.0"
NAME = "mpc_walk_path"

HORIZON = 25  # steps
DT = 0.1  # s, also the planner period
MAX_OBSTACLES = 4  # obstacle slots, unused ones are switched off

# Parameter vector layout, per stage. Mirrored into the generated mpc_walk_path_layout.h.
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
NP = P_ACTIVE + MAX_OBSTACLES


def f(pose, u):
    """Omnidirectional kinematics: the body twist u rotated into the solve frame."""
    c, s = ca.cos(pose[2]), ca.sin(pose[2])
    return ca.vertcat(c * u[0] - s * u[1], s * u[0] + c * u[1], u[2])


def rk4(pose, u, dt):
    k1 = f(pose, u)
    k2 = f(pose + dt / 2 * k1, u)
    k3 = f(pose + dt / 2 * k2, u)
    k4 = f(pose + dt * k3, u)
    return pose + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def build_ocp():
    from acados_template import AcadosModel, AcadosOcp

    m = MAX_OBSTACLES
    s = ca.SX.sym("s", 6)
    a = ca.SX.sym("a", 3)
    p = ca.SX.sym("p", NP)

    target = p[P_TARGET : P_TARGET + 3]
    gate, bearing = p[P_GATE], p[P_BEARING]
    w_position, w_heading, w_face, delta = p[P_W_POSITION], p[P_W_HEADING], p[P_W_FACE], p[P_HUBER_DELTA]
    w_effort, w_rate = p[P_W_EFFORT : P_W_EFFORT + 3], p[P_W_RATE : P_W_RATE + 3]
    radius = p[P_OBSTACLE_RADIUS]
    obstacles = ca.reshape(p[P_OBSTACLES : P_OBSTACLES + 2 * m], 2, m)
    active = p[P_ACTIVE : P_ACTIVE + m]

    u = s[3:] + a * DT
    model = AcadosModel()
    model.name = NAME
    model.x, model.u, model.p = s, a, p
    model.disc_dyn_expr = ca.vertcat(rk4(s[:3], u, DT), u)

    d2 = ca.sumsqr(s[:2] - target[:2])
    position = delta**2 * (ca.sqrt(1 + d2 / delta**2) - 1)
    # w_head·h(θ), written out so that w_head = 0 doesn't divide by zero
    heading = w_heading * gate * (1 - ca.cos(s[2] - target[2])) + w_face * (1 - gate) * (1 - ca.cos(s[2] - bearing))
    effort = ca.dot(w_effort, u**2)
    rate = ca.dot(w_rate, (a * DT) ** 2)
    # Stage k costs the pose s_k (reached by the previous step) and the command applied now. At k = 0 the pose is
    # fixed, so its term is a constant; the pose at N is costed by the terminal cost with the same weights.
    model.cost_expr_ext_cost = w_position * position + heading + effort + rate
    model.cost_expr_ext_cost_e = w_position * position + heading

    clearance = ca.vertcat(*[active[j] * (radius**2 - ca.sumsqr(s[:2] - obstacles[:, j])) for j in range(m)])
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

    # Placeholder limits: the module sets the real ones from its configuration before every solve
    ocp.constraints.idxbu = np.arange(3)
    ocp.constraints.lbu, ocp.constraints.ubu = -np.ones(3), np.ones(3)
    ocp.constraints.idxbx = np.array([3, 4, 5])
    ocp.constraints.lbx, ocp.constraints.ubx = -np.ones(3), np.ones(3)
    ocp.constraints.idxbx_e = np.array([3, 4, 5])
    ocp.constraints.lbx_e, ocp.constraints.ubx_e = -np.ones(3), np.ones(3)
    ocp.constraints.x0 = np.zeros(6)

    big = 1e9
    for suffix in ("", "_e"):
        setattr(ocp.constraints, "lh" + suffix, -big * np.ones(m))
        setattr(ocp.constraints, "uh" + suffix, np.zeros(m))
        setattr(ocp.constraints, "idxsh" + suffix, np.arange(m))
        setattr(ocp.cost, "zl" + suffix, np.zeros(m))
        setattr(ocp.cost, "zu" + suffix, 1000.0 * np.ones(m))  # set from the configuration's w_slack
        setattr(ocp.cost, "Zl" + suffix, np.zeros(m))
        setattr(ocp.cost, "Zu" + suffix, np.ones(m))

    so = ocp.solver_options
    so.integrator_type = "DISCRETE"
    so.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    so.hessian_approx = "EXACT"
    so.regularize_method = "MIRROR"
    so.levenberg_marquardt = 0.0
    # Without merit backtracking, SQP stalls on near-target approaches (walk-mpc's solver study)
    so.globalization = "MERIT_BACKTRACKING"
    so.nlp_solver_type = "SQP"
    so.nlp_solver_max_iter = 10  # set from the configuration's max_iterations
    so.tol = 1e-6
    so.print_level = 0
    return ocp


LAYOUT_HEADER = """\
/* Generated by codegen/generate_solver.py (acados {version}); do not edit. */
#ifndef MPC_WALK_PATH_LAYOUT_H
#define MPC_WALK_PATH_LAYOUT_H

#define MPC_WALK_PATH_DT {dt}
#define MPC_WALK_PATH_MAX_OBSTACLES {m}

/* Per-stage parameter vector layout */
{indices}

#endif /* MPC_WALK_PATH_LAYOUT_H */
"""


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

    names = {k: v for k, v in globals().items() if k.startswith("P_")}  # NP is in the solver header
    indices = "\n".join(f"#define MPC_WALK_PATH_{k} {v}" for k, v in names.items())
    (args.output / "mpc_walk_path_layout.h").write_text(
        LAYOUT_HEADER.format(version=ACADOS_VERSION, dt=DT, m=MAX_OBSTACLES, indices=indices)
    )
    print(f"Wrote {sum(1 for _ in args.output.rglob('*.[ch]'))} files to {args.output}")


if __name__ == "__main__":
    main()
