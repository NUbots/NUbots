# InvertedMPCWalkPath

## Description

Plans walk velocity commands to a target pose with model predictive control, as a drop-in for PlanWalkPath's `WalkTo`
(target pose in, `skill::Walk` velocity out). It is MPC-3: MPCWalkPath's kinematic MPC planning the velocity the robot
delivers, held inside the walk policy's capability envelope, with the command that delivers that velocity looked up
afterwards in the policy's measured response. Nothing about the policy's transients is modelled: the plan assumes the
robot delivers what it plans at once, and replanning every 0.1 s corrects for the difference. It only provides
`WalkTo`; PlanWalkPath still provides `TurnOnSpot`, `PivotAroundPoint` and `Adjust`, and it can't share a role with
PlanWalkPath, MPCWalkPath or ModelledMPCWalkPath, as they all provide `WalkTo`.

| Planner             | Plans                          | The policy's response                                        | Capability envelope |
| ------------------- | ------------------------------ | ------------------------------------------------------------ | ------------------- |
| MPCWalkPath         | the command, assumed delivered | PlanWalkPath's hand-tuned dead zone, after the solve         | no                  |
| ModelledMPCWalkPath | the command                    | an identified Hammerstein model, inside the solver           | on the command      |
| InvertedMPCWalkPath | the delivered velocity         | its measured steady-state response, inverted after the solve | on the velocity     |

Every time `WalkTo` is run (10 Hz under the Director), it solves, in the robot frame, over 25 steps of 0.1 s:

```
state   s = (x, y, θ, vx, vy, ω)      pose, and the velocity planned for the previous step
input   a = (ax, ay, aω)              the change of velocity per second
step    v = s[3:] + a·dt,  pose⁺ = RK4(pose, v, dt)        omnidirectional kinematics

minimise  Σₖ w_position·ρ(‖pₖ − p*‖)                          pseudo-Huber distance to the target
           + w_heading·gₖ·(1 − cos(θₖ − θ*))                   final heading, faded in near the target
           + w_face·(1 − gₖ)·(1 − cos(θₖ − βₖ))                facing the direction of travel far away
           + Σᵢ w_effort,i·vₖ,ᵢ² + Σᵢ w_rate,i·(aₖ,ᵢ·dt)²       effort and smoothness
           + slack penalties                                   soft envelope and obstacle clearance

subject to  lower ≤ v ≤ upper                                  the velocity limits, within what the commands delivered
            nⱼ·vₖ ≤ dⱼ + σₖⱼ,  σₖⱼ ≥ 0                          the capability envelope, soft
            ‖pₖ − oⱼ‖² ≥ obstacle_radius² − sₖⱼ,  sₖⱼ ≥ 0        other robots, soft
```

The cost, the warm start and the cold start are MPCWalkPath's (see its README). As there, the change of velocity is
unbounded, and `w_rate` across the whole horizon is what makes the plan slow down before the target.

The first planned velocity is then turned into a command:

1. Planned velocities smaller than `zero_tolerance` are taken as zero, so the robot stands still at the target rather
   than stepping out of the policy's standing region to correct a few millimetres.
2. The vx and vy command is the smallest one whose measured steady-state translation is the planned one, from the
   vx × vy grid. Then the ω command is the smallest whose measured turn rate, at that vx command, is the planned one,
   from the vx × ω grid. Where no command delivers the velocity, the one that comes closest is sent.

If a solve fails, or runs over `time_budget`, PlanWalkPath's `WalkTo` control law is sent instead, through
PlanWalkPath's dead zone and its limits on the command, and the next solve's rate cost starts from what that command
delivers.

## The measured response

`InvertedMPCWalkPathResponse.yaml` holds the velocity each command delivered in mjlab's evaluation of the policy
(`k1-poi732yy`, each command held for a minute): the vx × vy grid with no turn, and the vx × ω grid with no sideways
command, every 0.1 m/s and rad/s over the whole evaluated grid, ±2. Between grid commands it is interpolated
bilinearly. The velocity limits (`max_velocity`, `max_backward_velocity`) bound the planned velocity, not the command:
delivering a velocity at the limit can take a larger command (1.7 m/s forward takes about 1.9, 1.3 m/s sideways about
1.6). The inverse searches every 0.01 of command (about 160,000 translation commands and 400 turn commands, about
0.7 ms per solve), and takes the command nearest zero among those within
0.005 of the best match, as the response is flat where the policy stands still.

The grids show what the per-axis sweeps can't, as those were run walking (vy and ω at vx = 0.2 m/s): from standing, the
policy stands still for small commands on every axis together. With vx at 0 it doesn't step sideways below about
0.35 m/s, but with vx at 0.2 m/s it does, nearly linearly. Turning on the spot it barely turns below about 1 rad/s
(0.5 rad/s gives 0.06 rad/s, 1.0 gives 0.43), but walking forward its turn rate follows the command. The inverse finds
these jumps from the data: a small planned velocity from standing gets the command at the edge of the standing region.

## The capability envelope

The velocities the policy delivered for all the grids' commands, from the same evaluation's three grids: in each
plane, an octagon with fixed edge directions around them, whose 8 edges are linear constraints on the planned velocity,
so 24 in all (`envelope:` in `InvertedMPCWalkPath.yaml`). It cuts the combinations the policy doesn't deliver, mainly
walking backwards and sideways at once ((−1.5, 1.4) m/s commanded gives (−0.92, 0.71) m/s) and turning fast while
walking fast. The envelope is soft (`w_envelope`) and off at the first stage, so a velocity from outside it (after a
fallback) can't make the problem unsolvable. `envelope_scale` shrinks it towards standing still.

Both the response and the envelope are imported from the evaluation, which also writes the envelope:

```bash
uv run --no-project --with numpy python module/planning/InvertedMPCWalkPath/codegen/import_response.py \
    ~/Starman/mjlab_k1/booster_mjlab/logs/eval/k1-poi732yy
```

Things to know:

- Each grid holds the third axis fixed (the vy × ω grid at vx = 0.2 m/s), so the response leaves out vy's effect on
  turning and ω's on translation, and the envelope is the intersection of the three planes' octagons.
- The evaluation is the policy in mjlab with no randomisation, and steady states only: what the policy does while the
  command changes isn't in it, and NUSim or the robot may respond differently.
- The response is interpolated between grid commands 0.1 apart, so the edge of the standing region is only known to
  about 0.1: the vx sweep, every 0.025 m/s, puts it between 0.125 and 0.15 m/s.

## The solver

The problem is solved with [acados](https://github.com/acados/acados)' SQP (exact Hessian, HPIPM for the QPs, merit
backtracking, at most `max_iterations` iterations). The problem is C++ in `src/Problem.hpp`: its dynamics, costs and
obstacle constraints are templated on their scalar type, and [autodiff](https://autodiff.github.io) differentiates them
for the solver (`src/ocp/OcpFunctions.hpp`), which `src/ocp/AcadosOcp` sets up through acados' C interface. There is no
generated code. Only the horizon (25 × 0.1 s), the number of obstacle slots (4) and the number of envelope rows (24)
are fixed in `Problem.hpp`; everything else is set from `InvertedMPCWalkPath.yaml` and the response.

## Usage

Include this module in a role in place of PlanWalkPath, and emit a `WalkTo` Task.
`roles/nusim/walkpathbenchmark_inverted_mpc.role` and `roles/nusim/walktrackbenchmark_inverted_mpc.role` are the
benchmark roles with it.

## Consumes

- `message::planning::WalkTo` Task requesting to walk towards a given pose in robot space.
- `message::localisation::Robots` the robots to avoid.
- `message::input::Sensors` to transform robot points into (our own) robot space.
- `message::behaviour::state::Stability` to reset the MPC when the robot starts walking from standing or after a fall.

## Emits

- `message::skill::Walk` Task requesting to run the walk, giving the xy velocity (m/s) and rotational velocity
  (radians/s).
- `message::planning::WalkToDebug`, as PlanWalkPath does.
- NUsight DataPoints: "MPC Planned Velocity", "Walk Command" (sent), "MPC Solve Time (ms)", "MPC Iterations",
  "MPC Fallback", "MPC Envelope Violation" (of the planned velocity) and "MPC Horizon End" (the predicted pose at the
  end of the horizon).

## Dependencies

- Director
- acados, HPIPM and BLASFEO, and autodiff (built into the Docker image)
- PlanWalkPath's `walk_path_control.hpp`, for the fallback control law and dead zone
- The K1WalkPolicy skill (or another consumer of `skill::Walk`)
