# MPCWalkPath

## Description

Plans walk velocity commands to a target pose with model predictive control, as a drop-in for PlanWalkPath's `WalkTo`
(target pose in, `skill::Walk` velocity out). It is MPC-1 of the walk-mpc prototype: a kinematic MPC, ported with its
cost function and solver, using PlanWalkPath's velocity limits. It only provides `WalkTo`; PlanWalkPath still provides
`TurnOnSpot`, `PivotAroundPoint` and `Adjust`, and the two can't be in the same role, as both provide `WalkTo`.

Every time `WalkTo` is run (10 Hz under the Director), it solves, in the robot frame, over 25 steps of 0.1 s:

```
state   s = (x, y, θ, vx, vy, ω)      pose, and the command applied in the previous step
input   a = (ax, ay, aω)              the change of command per second
step    u = s[3:] + a·dt,  pose⁺ = RK4(pose, u, dt)        omnidirectional kinematics

minimise  Σₖ w_position·ρ(‖pₖ − p*‖)                          pseudo-Huber distance to the target
           + w_heading·gₖ·(1 − cos(θₖ − θ*))                   final heading, faded in near the target
           + w_face·(1 − gₖ)·(1 − cos(θₖ − βₖ))                facing the direction of travel far away
           + Σᵢ w_effort,i·uₖ,ᵢ² + Σᵢ w_rate,i·(aₖ,ᵢ·dt)²       effort and smoothness (replaces PlanWalkPath's smoothing)
           + slack penalty                                     soft obstacle clearance

subject to  −max_backward_velocity ≤ vx ≤ max_velocity.x,  |vy| ≤ max_velocity.y,  |ω| ≤ max_velocity.z
            ‖pₖ − oⱼ‖² ≥ obstacle_radius² − sₖⱼ,  sₖⱼ ≥ 0        other robots, soft
```

- ρ(d) = δ²(√(1 + d²/δ²) − 1) is quadratic within `huber_delta` of the target and linear beyond, which approximates
  minimum time.
- gₖ = exp(−d̂ₖ²/`heading_radius`²) fades the final heading in near the target, and βₖ is the bearing from the robot to
  the target. Both use d̂ₖ, the distance predicted by the initial guess (the previous solution shifted by one step), so
  they are fixed for each solve. Were they functions of the optimised path, walking away from the target would make
  heading errors cheaper, and the robot would swing out and back to turn on the spot.
- The target is treated as stationary over the horizon.

Only the first command is sent. The solve is warm started from the previous one, and from standing it starts with a
small turn towards the target, which breaks the tie when the target is exactly behind.

The command sent then goes through PlanWalkPath's dead zone (`dead_zone_compensation`), as the walk policy stands still
for commands too small to start stepping. The MPC itself knows nothing about the policy's response; planning with an
identified model of it is MPC-2. If a solve fails, or runs over `time_budget`, PlanWalkPath's `WalkTo` control law is
sent instead.

The change of command is unbounded: the walk policy keeps the robot stable for whatever it is commanded, so the
`w_rate` cost alone shapes it. That cost is needed on every step of the horizon. Tested on the walk-to-pose benchmark
in NUSim (2026-10-21), with no rate cost the MPC, which assumes the robot can stop instantly, reached the target at
1.3 m/s and settled 0.23 m from it (steady-state RMSE, against 0.15 m with `w_rate`); costing only the first step's
change, from the command sent last, let it put off every change to a later step, and it never settled at all.

## The solver

The problem is solved with [acados](https://github.com/acados/acados)' SQP (exact Hessian, HPIPM for the QPs, merit
backtracking, at most `max_iterations` iterations). The problem is C++ in `src/Problem.hpp`: its dynamics, costs and
obstacle constraints are templated on their scalar type, and [autodiff](https://autodiff.github.io) differentiates them
for the solver (`shared/utility/ocp/OcpFunctions.hpp`), which `utility::ocp::AcadosOcp` sets up through acados' C
interface. There is no generated code. Only the horizon (25 × 0.1 s) and the number of obstacle slots (4) are fixed in
`Problem.hpp`: the cost weights and obstacle radius are parameters, the limits are bounds, and the slack penalty and
iteration cap are solver settings, so all of them are set from `MPCWalkPath.yaml`.

On the walk-mpc prototype's scenarios against an ideal robot, the first command matches the prototype's acados MPC to
six decimals. Solves average about 5 ms on a desktop (about 2 ms when the derivatives were CasADi-generated code;
autodiff's Hessians recompute the trigonometry for every pair of variables).

## Usage

Include this module in a role in place of PlanWalkPath, and emit a `WalkTo` Task. `roles/nusim/walkpathbenchmark_mpc.role`
is the WalkPathBenchmark role with it.

## Consumes

- `message::planning::WalkTo` Task requesting to walk towards a given pose in robot space.
- `message::localisation::Robots` the robots to avoid.
- `message::input::Sensors` to transform robot points into (our own) robot space.
- `message::behaviour::state::Stability` to reset the MPC when the robot starts walking from standing or after a fall.

## Emits

- `message::skill::Walk` Task requesting to run the walk, giving the xy velocity (m/s) and rotational velocity
  (radians/s).
- `message::planning::WalkToDebug`, as PlanWalkPath does.
- NUsight DataPoints: "MPC Command" (before the dead zone), "Walk Command" (sent), "MPC Solve Time (ms)",
  "MPC Iterations", "MPC Fallback" and "MPC Horizon End" (the predicted pose at the end of the horizon).

## Dependencies

- Director
- acados, HPIPM and BLASFEO, and autodiff (built into the Docker image)
- PlanWalkPath's `walk_path_control.hpp`, for the fallback control law and the dead zone
- The K1WalkPolicy skill (or another consumer of `skill::Walk`)
