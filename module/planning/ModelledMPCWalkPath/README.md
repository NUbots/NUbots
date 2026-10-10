# ModelledMPCWalkPath

## Description

Plans walk velocity commands to a target pose with model predictive control, planning with an identified model of how
the walk policy responds to its command. A drop-in for PlanWalkPath's `WalkTo` (target pose in, `skill::Walk` velocity
out). It is MPCWalkPath (MPC-1) with the policy's response added, which is MPC-2 of the walk-mpc prototype. Like
MPCWalkPath, it only provides `WalkTo`, and can't be in a role with PlanWalkPath or MPCWalkPath, as they all provide
`WalkTo`.

MPCWalkPath plans as though the command were delivered. The K1 policy doesn't deliver it: it stands still for small
commands (dead zones of about ±0.35 m/s sideways and +0.18 m/s forward, and backward commands up to −0.15 m/s give
about −0.04 m/s), under-delivers some axes, couples them, and lags. So MPCWalkPath has to bump its commands out of the
dead zones afterwards (PlanWalkPath's `apply_dead_zone`), and its plan isn't what the robot does. This module plans
the commands themselves through the model, so the plan steps past a dead zone when that is the way to get somewhere and
needs no dead-zone shaping.

Every time `WalkTo` is run (10 Hz under the Director), it solves, in the robot frame, over 25 steps of 0.1 s:

```
state   s = (x, y, θ, z, u_prev)      pose, the model's 9 lag states, and the command applied in the previous step
input   a = (ax, ay, aω)              the change of command per second
step    u = u_prev + a·dt             command held over the step, then 5 model steps of Ts = 0.02 s:
          v = (Σⱼ z_vx,j, Σⱼ z_vy,j, Σⱼ z_ω,j)             delivered velocity
          pose⁺ = pose + Ts·(R(θ + ω·Ts/2)·(vx, vy), ω)
          zᵢⱼ⁺ = pᵢⱼ·zᵢⱼ + bᵢⱼ·fᵢⱼ(uⱼ)                       the model, one path per (output i, command j)

minimise  Σₖ w_position·ρ(‖pₖ − p*‖)                          pseudo-Huber distance to the target
           + w_heading·gₖ·(1 − cos(θₖ − θ*))                   final heading, faded in near the target
           + w_face·(1 − gₖ)·(1 − cos(θₖ − βₖ))                facing the direction of travel far away
           + Σᵢ w_effort,i·vₖ,ᵢ² + Σᵢ w_rate,i·(aₖ,ᵢ·dt)²       delivered speed, and smoothness of the command
           + slack penalty                                     soft obstacle clearance

subject to  −max_backward_velocity ≤ ux ≤ max_velocity.x,  |uy| ≤ max_velocity.y,  |uω| ≤ max_velocity.z
            nᵢ·u ≤ dᵢ + σₖᵢ,  σₖᵢ ≥ 0                         the capability envelope, soft
            |a| ≤ max_acceleration
            ‖pₖ − oⱼ‖² ≥ obstacle_radius² − sₖⱼ,  sₖⱼ ≥ 0
```

The costs are MPCWalkPath's (see its README for ρ, gₖ and βₖ), except that effort is on the delivered velocity rather
than the command. The limits are on the command: the policy's capability envelope (below).

## The model

A Hammerstein model identified in MATLAB (walkmodel-identification, branch hammerstein-model) on the poi732yy policy in
simulation: `idnlhw` with a piecewise-linear input map per path and first-order linear blocks with a one-step delay
(nb = nf = nk = 1), no output map and no standing gate. Each output (vx, vy, ω) is the sum of three paths, one per
command axis:

```
fᵢⱼ(u) = Lᵢⱼ·u + dᵢⱼ + Σₙ cᵢⱼₙ·|u + tᵢⱼₙ|          (6 breakpoints per path, at −tᵢⱼₙ)
zᵢⱼ⁺   = pᵢⱼ·zᵢⱼ + bᵢⱼ·fᵢⱼ(uⱼ)
```

It predicts the gait-averaged velocity; the sway within each stride isn't modelled.

The parameters are in the `model:` section at the end of `ModelledMPCWalkPath.yaml`, written by
`codegen/import_model.py` from the `.mat` the fit saves (it reads MATLAB's class objects with the mat-io package, so
MATLAB isn't needed):

```bash
uv run --no-project --with mat-io python module/planning/ModelledMPCWalkPath/codegen/import_model.py \
    ~/walkmodel-identification/hammerstein_no_gate_model_k1.mat --policy "poi732yy (k1_walk_mjlab_poi732yy_29999.onnx)"
```

A refitted model with the same structure (sample time, 6 breakpoints, first-order paths) only needs re-importing; the
import refuses anything else. The form of the piecewise-linear map isn't documented by MATLAB: it was the one whose
steady state matched the poi732yy policy's measured steady-state velocity grid (walk-mpc's
`data/k1_poi732yy_response.json`, RMSE 0.04 m/s, 0.07 m/s and 0.12 rad/s within |command| ≤ 1).

Things to know about this fit:

- The fitted ω map has two narrow non-monotonic notches, where a larger turn command turns slower (between −0.473 and
  −0.458, and between 0.437 and 0.484). They are local minima the solver gets stuck in: it held ω at −0.474 and turned
  the robot past the target heading. The solver's model smooths the maps' kinks, |x| ≈ √(x² + ε²), with
  `kink_smoothing` per command axis: 0.05 on ω merges the notches' breakpoints (the map moves by at most 0.03 rad/s),
  and 0.01 on vx and vy keeps their dead-zone edges sharp. The estimate below uses the unsmoothed model.
- With no gate, the model drifts at zero command (ω 0.018 rad/s, vy 0.003 m/s), so at the target the MPC holds the
  robot with a small command, about (0, −0.025, −0.026), well inside the policy's dead zones.
- The ω←vy path has a 15 s time constant (a pole at 0.9987), and vy←ω a negative pole (−0.34). Both carry little.

## The capability envelope

The commands the policy is known to deliver, from mjlab's evaluation of it (`k1-poi732yy-evaluation`: the vx × vy,
vx × ω and vy × ω command grids over ±2, each command held for a minute). Every command in the grids survived, so the
envelope is set by tracking instead: the commands whose steady-state planar velocity error is at most 0.2 m/s and yaw
rate error at most 0.2 rad/s. In each plane, an octagon with fixed edge directions is fitted around them, and its 8
edges are linear constraints on the command, so 24 in all (`envelope:` in `ModelledMPCWalkPath.yaml`). The octagons
take in the badly tracked commands near zero (the dead zones, and turning on the spot), which the model accounts for,
and cut the corners the policy doesn't deliver, mainly walking backwards and sideways at once: (−1.5, 1.5) gives
(−0.83, 0.62) m/s, where the model, whose axes simply add, predicts (−1.26, 1.21). The velocity limits are the
envelope's extent along each axis, clipped to the range the model was identified on (`identified_range`, beyond which
its maps extrapolate; past 1.72 m/s its vy map turns back down): vx from −1.5 to 1.75 m/s, vy ±1.4 m/s and ω ±1.5
rad/s.

```bash
uv run --no-project --with numpy python module/planning/ModelledMPCWalkPath/codegen/import_envelope.py \
    ~/mjlab/logs/eval/k1-poi732yy-evaluation
```

writes both (`--tolerance` and `--edges` change the fit). Things to know:

- The envelope is soft (`w_envelope`), so a command from outside it (the previous one, or the fallback's) can't make
  the problem unsolvable. It is off at the first stage, which is pinned to the command already sent.
- Each grid holds the third axis fixed (the vy × ω grid at vx = 0.2), so commands on all three axes at once weren't
  measured: the envelope is the intersection of the three planes' octagons.
- The evaluation is the policy in mjlab with no randomisation, and only the commands, not how fast they change, so
  `max_acceleration` is still MPCWalkPath's.
- Against the evaluation's steady-state sweeps, the model is within about 0.05 m/s on vx from −1.75 to 2 m/s and on
  vy outside its dead zone up to ±1.5 m/s, but it under-predicts turning above about 0.75 rad/s by 10–20 %, and its
  lateral dead zone (±0.33 m/s) doesn't appear in the sweep (0.25 m/s gives 0.2 m/s after a minute), so it probably
  stands for the slow start of walking sideways from standing in the identification runs.

## The estimate

The delivered velocity isn't measured: the model's 9 lag states are run from the commands sent (the MPC's, or the
fallback's), advanced by the time since the last solve before each solve. They are reset to zero (standing) when the
robot starts walking from standing or after a fall, on a new `WalkTo`, and after `stale_time` without one. Every run
the model was fitted on starts at rest, so it starts from zero too.

## The solver

acados's SQP (exact Hessian, HPIPM, merit backtracking, at most `max_iterations` iterations), from C code generated by
`codegen/generate_solver.py` and committed in `solver/`, as in MPCWalkPath. The horizon, the model's structure, the
number of envelope rows (24) and the obstacle slots are fixed by the generated code; the weights, limits, envelope,
model parameters, kink smoothing, slack penalties and iteration cap are set from `ModelledMPCWalkPath.yaml`. To change
the problem itself, edit the generator and regenerate:

```bash
uv run --project ~/walk-mpc python module/planning/ModelledMPCWalkPath/codegen/generate_solver.py
```

It needs casadi and the `acados_template` of the acados release the Docker image builds (v0.6.0).

The first command from standing matches the problem solved through acados's Python interface, and against the model
as the robot it reaches every test goal, including 0.3 m sidesteps and 0.15 m steps forward that sit inside the dead
zones. Solves take 3–22 ms on a desktop with the envelope (2–10 ms without). If a solve fails, or runs over
`time_budget`, PlanWalkPath's `WalkTo` control law is sent instead, held to the acceleration limits and then through
PlanWalkPath's dead zone, as PlanWalkPath would.

## In NUSim

WalkPathBenchmark's 13 trials (fresh NUSim per run, 10 October 2026):

| Planner                                                   | OK           | Total time   | Mean position error |
| --------------------------------------------------------- | ------------ | ------------ | ------------------- |
| PlanWalkPath                                              | 12/13        | 75.8 s       | 0.109 m             |
| MPCWalkPath                                               | 12/13        | 78.1 s       | 0.101 m             |
| ModelledMPCWalkPath, PlanWalkPath's limits                | 10/13, 7/13  | 75.7, 84.2 s | 0.141, 0.161 m      |
| ModelledMPCWalkPath, PlanWalkPath's limits, backward 0.35 | 11/13        | 74.9 s       | 0.091 m             |
| ModelledMPCWalkPath, capability envelope (two runs)       | 12/13, 11/13 | 51.0, 50.4 s | 0.080, 0.082 m      |

- With the envelope it is about a third quicker than the others and no less accurate, with no falls, using the whole
  envelope (commands up to 1.75 m/s forward, 1.5 m/s back, 1.4 m/s sideways and 1.5 rad/s) and never leaving it.
- With PlanWalkPath's limits, most failures were stalls short of a target behind the robot: the backward limit of
  0.15 m/s is inside the policy's backward dead zone, the model knows it (it predicts −0.04 m/s; NUSim's robot doesn't
  move at all), and the horizon is too short to see that turning round would be quicker. PlanWalkPath and MPCWalkPath
  don't stall there because PlanWalkPath's dead zone bumps small backward commands to −0.35 m/s, past their backward
  limit.
- The remaining failures end turning on the spot: WalkToFieldPosition stops the walk within 0.15 rad, and the robot,
  still turning, coasts out of the benchmark's 0.2 rad. The model's ω responds within about 0.02 s, so the MPC turns
  at speed until the last moment.
- The model was identified in mjlab, and NUSim's robot doesn't follow it exactly: it turned at about 0.94 rad/s for a
  1.5 rad/s command, where the model predicts 1.37. The estimate is open loop; correcting it with a measured velocity
  is a next step.

## Usage

Include this module in a role in place of PlanWalkPath, and emit a `WalkTo` Task.
`roles/nusim/walkpathbenchmark_modelled_mpc.role` is the WalkPathBenchmark role with it.

## Consumes

- `message::planning::WalkTo` Task requesting to walk towards a given pose in robot space.
- `message::localisation::Robots` the robots to avoid.
- `message::input::Sensors` to transform robot points into (our own) robot space.
- `message::behaviour::state::Stability` to reset the MPC when the robot starts walking from standing or after a fall.

## Emits

- `message::skill::Walk` Task requesting to run the walk, giving the xy velocity (m/s) and rotational velocity
  (radians/s).
- `message::planning::WalkToDebug`, as PlanWalkPath does.
- NUsight DataPoints: "Walk Command" (sent; WalkPathBenchmark records it), "MPC Delivered Velocity" (the model's
  estimate when the solve started), "MPC Solve Time (ms)", "MPC Iterations", "MPC Fallback", "MPC Envelope Violation"
  (how far the command sent is outside the envelope, ≤ 0 inside) and "MPC Horizon End" (the predicted pose at the end
  of the horizon).

## Dependencies

- Director
- acados, HPIPM and BLASFEO (built into the Docker image)
- PlanWalkPath's `walk_path_control.hpp`, for the fallback control law and dead zone
- The K1WalkPolicy skill running the policy the model was identified on (poi732yy)
