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
state   s = (x, y, θ, z, u_prev)      pose, the model's 10 lag states, and the command applied in the previous step
input   a = (ax, ay, aω)              the change of command per second
step    u = u_prev + a·dt             command held over the step, then 5 model steps of Ts = 0.02 s:
          v = (Σⱼ z_vx,j, Σⱼ z_vy,j, Σⱼ z_ω,j + z×)        delivered velocity
          pose⁺ = pose + Ts·(R(θ + ω·Ts/2)·(vx, vy), ω)
          zᵢⱼ⁺ = pᵢⱼ·zᵢⱼ + bᵢⱼ·fᵢⱼ(uⱼ)                       the model, one path per (output i, command j),
          z×⁺ = p×·z× + b×·f×(uω·|ux|)                      and one to ω from the turn scaled by the forward command

minimise  Σₖ w_position·ρ(‖pₖ − p*‖)                          pseudo-Huber distance to the target
           + w_heading·gₖ·(1 − cos(θₖ − θ*))                   final heading, faded in near the target
           + w_face·(1 − gₖ)·(1 − cos(θₖ − βₖ))                facing the direction of travel far away
           + Σᵢ w_effort,i·vₖ,ᵢ² + Σᵢ w_rate,i·(aₖ,ᵢ·dt)²       delivered speed, and smoothness of the command
           + slack penalty                                     soft obstacle clearance

subject to  −max_backward_velocity ≤ ux ≤ max_velocity.x,  |uy| ≤ max_velocity.y,  |uω| ≤ max_velocity.z
            nᵢ·u ≤ dᵢ + σₖᵢ,  σₖᵢ ≥ 0                         the capability envelope, soft
            ‖pₖ − oⱼ‖² ≥ obstacle_radius² − sₖⱼ,  sₖⱼ ≥ 0
```

The costs are MPCWalkPath's (see its README for ρ, gₖ and βₖ), except that effort is on the delivered velocity rather
than the command. The limits are on the command: the policy's capability envelope (below). As in MPCWalkPath, the
change of command is unbounded, and only `w_rate` shapes it.

## The model

A Hammerstein model of the walk policy identified in MATLAB (walkmodel-identification): `idnlhw`
with a piecewise-linear input map per path and first-order linear blocks with a one-step delay (nb = nf = nk = 1), no
output map and no standing gate. Each output (vx, vy, ω) is the sum of its paths, one per command axis (a SISO fit has
only each output's own; the others are zero), and ω has one more, from the product ω·|vx| of the commands:

```
fᵢⱼ(u) = Lᵢⱼ·u + dᵢⱼ + Σₙ cᵢⱼₙ·|u + tᵢⱼₙ|          (6 breakpoints per path, at −tᵢⱼₙ)
zᵢⱼ⁺   = pᵢⱼ·zᵢⱼ + bᵢⱼ·fᵢⱼ(uⱼ)
z×⁺    = p×·z× + b×·f×(uω·|ux|)
```

It predicts the gait-averaged velocity; the sway within each stride isn't modelled.

The parameters are in `ModelledMPCWalkPathModel.yaml`, which walkmodel-identification's `fit_SISO_hammerstein.m`
writes next to the `.mat` it saves (`functions/exportHammersteinYaml.m`). To use a new fit, copy that file over
`data/config/ModelledMPCWalkPathModel.yaml`. An existing `.mat` is exported in MATLAB with

```matlab
exportHammersteinYaml("ModelledMPCWalkPathModel.yaml", load("hammerstein_siso_model_k1v2_27combinedRuns_wzProduct.mat"))
```

A refitted model with the same structure (sample time, 6 breakpoints, first-order paths) needs no regeneration; the
export refuses anything else, including a Hammerstein-Wiener model (one with an output map), which the generated solver
doesn't have. The map's form is idPiecewiseLinear's stored one (its `soevaluate1.m`), and the export checks the written
parameters reproduce the model's simulation. To combine outputs from different fits, replace entries of `models` before
exporting. Nothing checks that the model was identified on the policy K1WalkPolicy runs: matching them is up to you.

The model is `hammerstein_siso_model_k1v2_27combinedRuns_wzProduct.mat`, fitted by `fit_SISO_hammerstein.m` on the
k1v2_27combinedRuns dataset: SISO (each output from its own command, so paths 0, 4 and 8, with time constants of
0.36 s, 0.29 s and 0.02 s), and the ω·|vx| path (0.16 s). The policy turns in two regimes: standing, it barely turns at
small commands, and walking, it delivers most of the command. A single ω map can't fit both, and SISO fits without the
product dipped where they split the difference (a larger command turning slower), which stalled the solver. The product
path lets the walking regime be modelled separately: the model's steady ω for a 0.5 rad/s command is 0.17 rad/s on the
spot and 0.50 rad/s walking at 0.5 m/s. Having no cross paths, it doesn't know that walking in two directions at once
delivers less of each; the capability envelope cuts the worst of those commands.

Things to know about this fit:

- The solver's model smooths the maps' kinks, |x| ≈ √(x² + ε²), with `kink_smoothing` per command axis, because a
  non-monotonic notch in a map is a local minimum the solver gets stuck in. 0.05 on ω (also the product path's map) and
  0.01 on vx keep the maps within a few hundredths of the fit and the dead-zone edges sharp. The product's |vx| is
  smoothed with vx's width. The estimate below uses the unsmoothed model.
- The vy map has a notch between −0.326 and −0.243 m/s (−0.3 m/s delivers less than −0.15). 0.06 is the narrowest vy
  smoothing that removes it: at 0.03 a walk could stall 13 cm off its target with a −0.31 m/s sidestep. The cost is a
  softer vy dead zone in the solver's model (0.3 m/s delivers 0.085 m/s, against the fit's 0.066).
- The product path's map falls beyond |ω·vx| ≈ 0.84, so at a 1.5 rad/s turn the model turns slower walking at 1 m/s
  (1.34 rad/s) than at 0.5 m/s (1.59 rad/s).
- With no gate, the model drifts at zero command (−0.006 m/s forward, 0.008 m/s sideways and −0.016 rad/s), so at the
  target the MPC holds the robot with a small command inside the policy's dead zones.
- From cold, the solver's initial guess turns gently towards the target only when it has to turn round (more than 90°).
  A smaller turn isn't guessed: the policy delivers it, and a solve at the iteration cap would follow it off a straight
  walk.

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
its maps extrapolate): vx from −1.5 to 1.75 m/s, vy ±1.4 m/s and ω ±1.5 rad/s. `envelope_scale` shrinks the envelope
towards standing still (each bound times it); 0.95 made no measurable difference in NUSim, so it is 1.

```bash
uv run --no-project --with numpy python module/planning/ModelledMPCWalkPath/codegen/import_envelope.py \
    ~/mjlab/logs/eval/k1-poi732yy-evaluation
```

writes both (`--tolerance` and `--edges` change the fit). Things to know:

- The envelope is soft (`w_envelope`), so a command from outside it (the previous one, or the fallback's) can't make
  the problem unsolvable. It is off at the first stage, which is pinned to the command already sent.
- Each grid holds the third axis fixed (the vy × ω grid at vx = 0.2), so commands on all three axes at once weren't
  measured: the envelope is the intersection of the three planes' octagons.
- The evaluation is the policy in mjlab with no randomisation, and only the commands, not how fast they change. The
  change of command is unbounded, as in MPCWalkPath.
- Against the evaluation's steady-state sweeps, the model is within about 0.05 m/s on vx from −1.75 to 1.75 m/s and
  within 0.05 m/s on vy outside its dead zone up to ±1.25 m/s (0.1 m/s at ±1.5), and its
  lateral dead zone (±0.33 m/s) doesn't appear in the sweep (0.25 m/s gives 0.2 m/s after a minute), so it probably
  stands for the slow start of walking sideways from standing in the identification runs.

## The estimate

The delivered velocity isn't measured: the model's 10 lag states are run from the commands sent (the MPC's, or the
fallback's), advanced by the time since the last solve before each solve. They are reset to zero (standing) when the
robot starts walking from standing or after a fall, on a new `WalkTo`, and after `stale_time` without one. Every run
the model was fitted on starts at rest, so it starts from zero too.

## The solver

acados's SQP (exact Hessian, HPIPM, merit backtracking, at most `max_iterations` iterations), from C code generated by
`codegen/generate_solver.py` and committed in `solver/`, as in MPCWalkPath. The horizon, the model's structure, the
number of envelope rows (24) and the obstacle slots are fixed by the generated code; the weights, limits, envelope,
kink smoothing, slack penalties and iteration cap are set from `ModelledMPCWalkPath.yaml`, and the model parameters
from `ModelledMPCWalkPathModel.yaml`. To change
the problem itself, edit the generator and regenerate:

```bash
uv run --project ~/walk-mpc python module/planning/ModelledMPCWalkPath/codegen/generate_solver.py
```

It needs casadi and the `acados_template` of the acados release the Docker image builds (v0.6.0).

The first command from standing matches the problem solved through acados's Python interface, and against the model
as the robot it reaches every test goal, including 0.3 m sidesteps and 0.15 m steps forward that sit inside the dead
zones. Solves take 3–22 ms on a desktop with the envelope (2–10 ms without), and about 27 ms on average in
WalkPathBenchmark alongside NUSim. The SQP stops before an iteration that would take it past `time_budget` and its
iterate is sent, as at the iteration cap. If a solve fails, stops before its first iteration, or overshoots anyway (an
iteration can't be interrupted), PlanWalkPath's `WalkTo` control law is sent instead, through PlanWalkPath's dead zone,
as PlanWalkPath would.

## In NUSim

WalkPathBenchmark's 13 trials, a fresh NUSim per run. A trial passes once the robot stays within 0.2 m and 0.2 rad of
the target for 2 s, and times out after 40 s. With WalkToFieldPosition's default thresholds (it stops the walk within
0.15 m and rad, and only walks again beyond 0.4), every planner lost turns on the spot: stopped mid-turn, the K1 swings
up to 0.3 rad back against the turn as it stands, and nothing restarts it. These runs stop at 0.1 and restart at 0.2,
set per task through WalkToFieldPosition's threshold overrides (11–12 October 2026):

| Planner                                       | Runs | OK    | Total time, runs with no failure |
| --------------------------------------------- | ---- | ----- | -------------------------------- |
| PlanWalkPath                                  | 4    | 52/52 | 74.5–75.7 s                      |
| MPCWalkPath                                   | 4    | 52/52 | 77.0–78.7 s                      |
| ModelledMPCWalkPath, noChirp_noRamp MISO on ω | 3    | 37/39 | 51.7 s                           |
| ModelledMPCWalkPath, k1v2 SISO + ω·\|vx\|     | 8    | 95/97 | 48.5–52.9 s                      |

- It is about a third quicker than the others, with the same position error at the end of a trial (0.06–0.08 m) and
  no falls, using the whole envelope and never leaving it. One k1v2 run was cut short by a K1WalkPolicy data race,
  since fixed, so it counts the 97 trials completed; the three k1v2 runs since the fix passed all 39.
- Its remaining failures are turns on the spot that loop: near the target its heading cost, 1 − cos, is nearly flat,
  so it slows the turn and reaches the stop line still turning, the stand carries it back past 0.2 rad, and it starts
  again, until the trial times out. PlanWalkPath turns at its minimum rate to the end and stops within 0.02 rad.
- With PlanWalkPath's limits (the first fit, default thresholds), most failures were stalls short of a target behind
  the robot: the backward limit of 0.15 m/s is inside the policy's backward dead zone, the model knows it, and the
  horizon is too short to see that turning round would be quicker.
- The model was identified in mjlab, and NUSim's robot doesn't follow it exactly. The estimate is open loop;
  correcting it with a measured velocity is a next step.

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
- The K1WalkPolicy skill, running the policy the model was identified on
