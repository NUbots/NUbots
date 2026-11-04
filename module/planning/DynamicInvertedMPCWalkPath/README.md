# DynamicInvertedMPCWalkPath

## Description

Plans walk velocity commands to a target pose with model predictive control, as a drop-in for PlanWalkPath's `WalkTo`
(target pose in, `skill::Walk` velocity out). It is MPC-4: InvertedMPCWalkPath's MPC (MPC-3), planning the velocity the
robot delivers inside the walk policy's capability envelope, with the command found afterwards from the Hammerstein
model ModelledMPCWalkPath (MPC-2) plans with, and the model's estimate of what the robot is delivering now. Where
MPC-3's inverse is the steady state (the command that delivers the velocity once the robot has settled), this one is
dynamic: the command that, held from the estimate, delivers the planned velocity one planner step (0.1 s) later. It
only provides `WalkTo`; PlanWalkPath still provides `TurnOnSpot`, `PivotAroundPoint` and `Adjust`, and it can't share
a role with PlanWalkPath, MPCWalkPath, ModelledMPCWalkPath or InvertedMPCWalkPath, as they all provide `WalkTo`.

| Planner                    | Plans                  | The policy's response                                                       |
| -------------------------- | ---------------------- | --------------------------------------------------------------------------- |
| ModelledMPCWalkPath        | the command            | the Hammerstein model, inside the solver, over the whole horizon            |
| InvertedMPCWalkPath        | the delivered velocity | the measured response or the model's steady state, inverted after the solve |
| DynamicInvertedMPCWalkPath | the delivered velocity | the model and its estimate of the robot, inverted one step ahead            |

The MPC, its cost, limits, envelope, warm and cold starts are InvertedMPCWalkPath's (see its README). Every time
`WalkTo` is run (10 Hz under the Director):

1. The model's estimate of the robot is advanced by the time since the last solve, with the command sent then held.
2. The MPC plans the delivered velocity, as InvertedMPCWalkPath's does.
3. Planned velocities smaller than `zero_tolerance` are taken as zero, and the command is the smallest one that, held
   from the estimate for 0.1 s, delivers the planned velocity by the model (below).
4. The command sent is recorded, for the estimate to run on.

If a solve fails, or runs over `time_budget`, PlanWalkPath's `WalkTo` control law is sent instead, through
PlanWalkPath's dead zone and its limits on the command, and the next solve's rate cost starts from what that command
delivers by the next solve.

## The inverse

The model (`DynamicInvertedMPCWalkPathModel.yaml`, a copy of `ModelledMPCWalkPathModel.yaml`; the code is
`utility::planning::HammersteinModel`) is a first-order lag per path, z⁺ = p·z + b·f(u), with the paths of each output
summed. Held at a command u for n samples from lag states z, each path reaches

```
z(n) = pⁿ·z + b·(1 − pⁿ)/(1 − p)·f(u)
```

so one planner step ahead (n = 5 samples of 0.02 s) the delivered velocity is the estimate's free response plus each
path's rise for its command. The rises of the translation paths are tabulated once for every command on the grid
(`command_grid_step`, 0.01, out to the model's `identified_range`: vx and vy ±1.75, ω ±1.5), so the translation search
over every (vx, vy) command with no turn is additions only. The search is then InvertedMPCWalkPath's: the closest
translation command, a finer search around it with the model itself (a twentieth of a step apart, as the model's
dead zones end in steep edges: vy rises 15 times as fast as its command between 0.42 and 0.45 m/s), then the turn given
that forward command, likewise; of the commands within 0.005 of the closest, the one nearest zero. About 0.6 ms per
solve on a desktop.

The estimate is open loop, as ModelledMPCWalkPath's is: the lag states are run from the commands sent (the inverse's
or the fallback's), and reset to standing when the robot starts walking from standing or after a fall, on a new
`WalkTo`, after `stale_time` without one, and on a new configuration or model.

Things to know:

- A first-order lag only covers part of a change in 0.1 s (vx's about a quarter, vy's about 30 %, ω's nearly all), so
  the inverse is a deadbeat controller on the model: it leads with a command past the one that would hold the velocity
  (from 0.5 to 0.6 m/s forward takes a command that would settle above 0.8 m/s), and asks for the edge of the
  identified range for changes it can't make in one step. The change of command is unbounded, as in the other MPCs.
- The MPC still plans as though the velocity were delivered at once, from the velocity it planned last, not from the
  estimate; where the inverse can't keep up (at the edge of the range), the plan and the robot part until replanning
  catches up.
- The search is translation first, then the turn. Beyond |ω·vx| ≈ 0.84 of command the model's product path turns less
  for a larger command, so where speeding up takes a large vx command the turn asked for may be out of reach, and the
  turn command that comes closest is sent.
- The model drifts at the zero command, so at the target the inverse holds the robot with small commands inside the
  dead zones, as ModelledMPCWalkPath does.
- The velocity limits are what the model settles to within its identified range (1.61 m/s forward rather than 1.75).
- The model was identified in mjlab, and NUSim's robot doesn't follow it exactly; the estimate isn't corrected with a
  measured velocity.

## In NUSim

One run of WalkPathBenchmark (2026-11-04, a fresh NUSim, with InvertedMPCWalkPath's tuning: `w_face` 3 and
`heading_radius` 0.8): 13 of 13, no falls and no fallbacks, 42.8 s, mean errors 0.010 m and 0.025 rad, and paths
1.06–1.27 times the straight line on the trials that move.

## Usage

Include this module in a role in place of PlanWalkPath, and emit a `WalkTo` Task.
`roles/nusim/walkpathbenchmark_dynamic_inverted_mpc.role` and `roles/nusim/walktrackbenchmark_dynamic_inverted_mpc.role`
are the benchmark roles with it.

## Consumes

- `message::planning::WalkTo` Task requesting to walk towards a given pose in robot space.
- `message::localisation::Robots` the robots to avoid.
- `message::input::Sensors` to transform robot points into (our own) robot space.
- `message::behaviour::state::Stability` to reset the MPC and the estimate when the robot starts walking from standing
  or after a fall.

## Emits

- `message::skill::Walk` Task requesting to run the walk, giving the xy velocity (m/s) and rotational velocity
  (radians/s).
- `message::planning::WalkToDebug`, as PlanWalkPath does.
- NUsight DataPoints: "MPC Planned Velocity", "MPC Delivered Velocity" (the estimate when the solve started),
  "MPC Predicted Velocity" (what the command sent delivers by the next solve, by the model), "Walk Command" (sent),
  "MPC Solve Time (ms)", "MPC Iterations", "MPC Fallback", "MPC Envelope Violation" (of the planned velocity) and
  "MPC Horizon End" (the predicted pose at the end of the horizon).

## Dependencies

- Director
- acados, HPIPM and BLASFEO, and autodiff (built into the Docker image)
- `utility::planning::HammersteinModel`
- PlanWalkPath's `walk_path_control.hpp`, for the fallback control law and dead zone
- The K1WalkPolicy skill, running the policy the model was identified on
