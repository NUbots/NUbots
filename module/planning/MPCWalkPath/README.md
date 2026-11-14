# MPCWalkPath

## Description

Plans walk velocity commands to a target pose with model predictive control, as a drop-in for PlanWalkPath's `WalkTo`
(target pose in, `skill::Walk` velocity out). A kinematic MPC plans the velocity the robot is to deliver, and the
command that delivers it is found afterwards from a Hammerstein model of the walk policy's response and the model's
estimate of what the robot is delivering now: the command that, held from the estimate, delivers the planned velocity
one planner step (0.1 s) later. It only provides `WalkTo`; PlanWalkPath still provides `TurnOnSpot`,
`PivotAroundPoint` and `Adjust`, and the two can't be in the same role, as both provide `WalkTo`.

Every time `WalkTo` is run (10 Hz under the Director):

1. The model's estimate of the robot is advanced by the time since the last solve, with the command sent then held.
2. The MPC plans the delivered velocity (below).
3. Planned velocities smaller than `zero_tolerance` are taken as zero, so the robot stands still at the target rather
   than stepping to correct a few millimetres, and the command is the smallest one that, held from the estimate for
   0.1 s, delivers the planned velocity by the model (see [The inverse](#the-inverse)).
4. The command sent is recorded, for the estimate to run on.

If a solve fails, or runs over `time_budget`, PlanWalkPath's `WalkTo` control law is sent instead, through
PlanWalkPath's dead zone and its limits on the command, and the next solve's rate cost starts from what that command
delivers by the next solve.

## The MPC

It solves, in the robot frame, over 25 steps of 0.1 s:

```
state   s = (x, y, θ, vx, vy, ω)      pose, and the velocity planned for the previous step
input   a = (ax, ay, aω)              the change of velocity per second
step    v = s[3:] + a·dt,  pose⁺ = RK4(pose, v, dt)        omnidirectional kinematics

minimise  Σₖ w_position·ρ(‖pₖ − p*‖)                          pseudo-Huber distance to the target
           + w_heading·gₖ·(1 − cos(θₖ − θ*))                   final heading, faded in near the target
           + w_face·(1 − gₖ)·(1 − cos(θₖ − βₖ))                facing the direction of travel far away
           + Σᵢ w_effort,i·vₖ,ᵢ² + Σᵢ w_rate,i·(aₖ,ᵢ·dt)²       effort and smoothness
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
- The velocity limits are per axis, on the planned velocity rather than the command, and are clipped to what the model
  settles to within its identified range (1.61 m/s forward rather than 1.75). As they are a box, the MPC can plan
  combinations the policy tracks poorly, mainly backwards and sideways at once.
- With `w_face` at 0 the heading is free far from the target, and the MPC crabs for the box's extra diagonal speed and
  walks backwards rather than turn around.

Only the first planned velocity is used. The solve is warm started from the previous one, and from standing it starts
with a small turn towards the target, which breaks the tie when the target is exactly behind.

The change of velocity is unbounded: the walk policy keeps the robot stable for whatever it is commanded, so the
`w_rate` cost alone shapes it. The MPC plans as though the robot delivers what it plans at once, so that cost, on every
step of the horizon, is what makes the plan slow down before the target.

## The solver

The problem is solved with [acados](https://github.com/acados/acados)' SQP (exact Hessian, HPIPM for the QPs, merit
backtracking, at most `max_iterations` iterations). The problem is C++ in `src/Problem.hpp`: its dynamics, costs and
obstacle constraints are templated on their scalar type, and [autodiff](https://autodiff.github.io) differentiates them
for the solver (`shared/utility/ocp/OcpFunctions.hpp`), which `utility::ocp::AcadosOcp` sets up through acados' C
interface. There is no generated code. Only the horizon (25 × 0.1 s) and the number of obstacle slots (4) are fixed in
`Problem.hpp`: the cost weights and obstacle radius are parameters, the limits are bounds, and the slack penalty and
iteration cap are solver settings, so all of them are set from `MPCWalkPath.yaml`.

## The inverse

The model (`MPCWalkPathModel.yaml`; the code is `utility::planning::HammersteinModel`) is a first-order lag per path,
z⁺ = p·z + b·f(u), with the paths of each output summed. Held at a command u for n samples from lag states z, each path
reaches

```
z(n) = pⁿ·z + b·(1 − pⁿ)/(1 − p)·f(u)
```

so one planner step ahead (n = 5 samples of 0.02 s) the delivered velocity is the estimate's free response plus each
path's rise for its command. The rises of the translation paths are tabulated once for every command on the grid
(`command_grid_step`, 0.01, out to the model's `identified_range`: vx and vy ±1.75, ω ±1.5), so the translation search
over every (vx, vy) command with no turn is additions only. The search is the closest translation command, a finer
search around it with the model itself (a twentieth of a step apart, as the model's dead zones end in steep edges: vy
rises 15 times as fast as its command between 0.42 and 0.45 m/s), then the turn given that forward command, likewise;
of the commands within 0.005 of the closest, the one nearest zero. About 0.6 ms per solve on a desktop.

The estimate is open loop: the lag states are run from the commands sent (the inverse's or the fallback's), and reset
to standing when the robot starts walking from standing or after a fall, on a new `WalkTo`, after `stale_time` without
one, and on a new configuration or model.

Things to know:

- A first-order lag only covers part of a change in 0.1 s (vx's about a quarter, vy's about 30 %, ω's nearly all), so
  the inverse is a deadbeat controller on the model: it leads with a command past the one that would hold the velocity
  (from 0.5 to 0.6 m/s forward takes a command that would settle above 0.8 m/s), and asks for the edge of the
  identified range for changes it can't make in one step.
- The MPC still plans as though the velocity were delivered at once, from the velocity it planned last, not from the
  estimate; where the inverse can't keep up (at the edge of the range), the plan and the robot part until replanning
  catches up.
- The search is translation first, then the turn. Beyond |ω·vx| ≈ 0.84 of command the model's product path turns less
  for a larger command, so where speeding up takes a large vx command the turn asked for may be out of reach, and the
  turn command that comes closest is sent.
- The model drifts at the zero command, so at the target the inverse holds the robot with small commands inside the
  dead zones.
- The model was identified in mjlab on the walk policy `k1-poi732yy`, and NUSim's robot doesn't follow it exactly; the
  estimate isn't corrected with a measured velocity. A new walk policy needs a new model.

## Usage

Include this module in a role in place of PlanWalkPath, and emit a `WalkTo` Task.

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
  "MPC Solve Time (ms)", "MPC Iterations", "MPC Fallback" and "MPC Horizon End" (the predicted pose at the end of the
  horizon).

## Dependencies

- Director
- acados, HPIPM and BLASFEO, and autodiff (built into the Docker image)
- `utility::planning::HammersteinModel`
- PlanWalkPath's `walk_path_control.hpp`, for the fallback control law and dead zone
- The Walk skill, running the policy the model was identified on
