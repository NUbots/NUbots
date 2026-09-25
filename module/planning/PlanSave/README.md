# PlanSave

## Description

The goalie's save planner. It predicts shots from the ball estimate and runs the block policy (`skill::K1BlockPolicy`) when one is coming. While none is, it walks the goalie to where the policy is most likely to save the next shot. It uses the policy's measured **capability envelope** for both: to say how likely a block is to work, and to choose where to stand.

Every tick (50 Hz) it:

1. **Predicts:** takes our own ball estimate (`rBWw`, `vBw` and their covariance) and rolls it forward with constant rolling deceleration, to where it crosses:
   - the goalie's frontal plane (x = 0 in the robot frame `{r}`): `dy`, time to arrival, and their standard deviations;
   - our goal line: where along it, and `P(on target)`, the probability the ball goes between the posts.

   The prediction is `ShotCommand._predict_crossing` from the mjlab goalkeeper task, because the policy was trained on commands computed that way. The covariance is carried through a numerical Jacobian. `kick_widening` can widen the velocity covariance for the first moments of a shot. It is off: through the full stack in NUSim, the filter's covariance at the commit was already far wider than the actual error.
2. **Scores the block:** averages the envelope's conservative save rate (a Wilson lower bound per cell) over the Gaussian on `dy`, at the predicted time and speed. Probability outside the measured grid counts as a miss.
3. **Chooses:**
   - **IDLE:** walks the goalie to its chosen spot, facing the ball (see [Positioning](#positioning)). Without a spot, it emits nothing, so the positioning walk (a lower-priority sibling of `Save`) runs.
   - **GUARD:** the ball is within `guard_distance` and at least `guard_min_ahead` in front of the goalie's line. Emits an inactive `Block`, so the goalie holds the policy's ready stance in CUSTOM mode, which is the state the envelope was measured from.
   - **BLOCK:** a shot reaches the goalie's line within `max_time` and has `P(on target) ≥ min_p_on_target`. Emits `Block{active, dy, t, v}` every tick. The command follows the training rule: active while the ball is on its way, zeros otherwise. PlanSave sticks with the block until the shot is over: the ball is no longer on its way (stopped, past the goalie or going away) or lost, for `release_delay`.

   A **walking goalie is handed to the block policy (GUARD or BLOCK) only with both feet down**, for at most `handoff.max_wait`. Until then it carries on as in IDLE. The policy was trained from a standing start, and taking over mid-stride topples it. The K1 has no foot contact sensors, so "both feet down" is the two ankles level to within `handoff.foot_height_tolerance`, from the leg joints (the K1 URDF's leg chain) and the IMU. In NUSim the ankles come level every 0.18 s (median) while walking. `SavePlan` carries the ankle-height difference and the wait.

   The block is the only skill so far. PlanSave still blocks a shot whose expected success is under `block_threshold`, but warns: that is the gap a dive policy would fill.

Each tick is published as `message::planning::SavePlan`: the prediction, its uncertainty, `P(on target)`, the expected save rate and fall rate, the mode, and the goalie's chosen spot. Shots are also logged at INFO when they start and end, so the predictions can be matched against the outcomes.

## Positioning

While `Save` runs and the goalie isn't blocking, PlanSave chooses where it should stand, 5 times a second. `tools/policy/save_positioning_prototype.py` has the derivation, the plots and the reference results `tests/TestPositioning.cpp` holds `src/positioning.hpp` to. Change the two together.

For the ball where it is now, every straight shot at the goal lies in the triangle between the ball and the posts. Every spot on a `grid_step` grid, from `min_depth` off the goal line out to the **penalty area line** and across the penalty area, is scored against those shots with the goalie facing the ball:

1. Each shot (41 aim points between the posts × 11 speeds from `speed_range`) that reaches the goal line crosses the goalie's frontal plane at some `dy` and time. The policy's smoothed capability `S(dy, t, v)` (`SaveCapability.yaml`) says how likely it saves it, with `t_lat` taken off the time.
2. A spot's score is its save rate over the worst `cvar_fraction` of aim points (`objective: cvar`, a shooter who picks the goalie's gaps), or over all of them (`mean`).
3. It is scored again on guesses at how much worse the real goalie is than mjlab's: every combination of an extra delay (`extra_lat`) and a reach scale (`reach_scale`, a shot at `dy` scored as mjlab's shot at `dy / scale`). With `robust: regret`, the spot is chosen on its worst-case regret: how far short of each guess's own best spot it falls, at most.
4. The goalie goes to the spot **closest to the goal line** within `near_best` of the best. The score is nearly flat in depth for central balls, so the maximum's depth is noise. It keeps going to the spot it already has while that is still within `near_best`, so the target doesn't hop between spots that are as good as each other.

No spot comes out if no shot from the ball reaches the goal, if the ball is lost, or if `SaveCapability.yaml` doesn't match the deployed policy. PlanSave then emits nothing in IDLE, and the walk below `Save` (`purpose::Goalie`'s strafe) positions the goalie. `positioning.enabled: false` does the same always, for comparisons.

Choosing a spot takes about 20 ms on a desktop CPU (2,000 spots × 451 shots × 4 guesses), which is why it is not done every tick. `SavePlan` carries the spot (`rTGg`, in the goal frame), its save rates, regret and how long it took.

## The envelope and its policy

`data/config/SaveEnvelope.yaml` holds the envelope. It counts on-target shots, saves and falls over signed `dy` × time to arrival × ball speed. `data/config/SaveCapability.yaml` holds the same shots smoothed onto a fine grid for positioning: Gaussian-smoothed, shrunk towards 0 where there are few shots, and filled so that more time or a slower ball never lowers the save rate. Both record the SHA-256 of the ONNX they were measured from. PlanSave hashes the file named by `model_path` in `K1BlockPolicy.yaml`. **If the envelope doesn't match, or either is missing, PlanSave never blocks, and the goalie only positions. If the capability doesn't match, positioning is left to the walk.** Replacing the policy therefore means re-measuring both:

```sh
# In mjlab, at the commit the policy was trained at
uv run src/mjlab/tasks/goalkeeper/scripts/measure_envelope.py --checkpoint logs/rsl_rl/k1_block/<run>/model_<n>.pt --num-envs 1024 --steps 6000
# Here
# (writes SaveCapability.yaml beside it too; needs numpy, scipy and PyYAML, as in mjlab's environment)
python3 tools/policy/make_save_envelope.py <run>/envelope.csv -o module/planning/PlanSave/data/config/SaveEnvelope.yaml --name "<run>"
```

The current envelope is the standing-start retrain (wandb `ckfpijgo`, `model_4999`, trained and measured at mjlab `bb0bd9951`): 25,777 shots from the task's "full" level. Crossings are within ±0.8 m of the keeper, speeds 1.5–4 m/s, from 2–4.5 m. It saved 63.6% of the on-target shots inside the grid, and fell during 4.1% (run 12: 69.9% and 5.9%). `measure_envelope` writes shots that end in a fall as off target, so `make_save_envelope.py` counts them back in as failed on-target shots (see its docstring).

## Usage

- Add `planning::PlanSave` and `skill::K1BlockPolicy` to the role. PlanSave reads `K1BlockPolicy.yaml`.
- Emit a `message::planning::Save` Task while the goalie guards the goal. Give it a higher priority than the positioning walk, so a `Block` it emits takes the servos from the walk. `purpose::Goalie` does this while it defends. `purpose::Tester` has `plan_save_priority`.

## Consumes

- `message::planning::Save`: Director task, guard the goal.
- `message::localisation::Ball`: our own estimate (confidence > 0) with covariance, for shots. Teammates' balls carry no velocity and are ignored for shots, but positioning needs only where the ball is, so it uses them too.
- `message::input::Sensors` (`Hrw`), `message::localisation::Field` (`Hfw`), `message::support::FieldDescription`.
- `message::platform::RawSensors` (leg joints and IMU, for both feet down) and `message::behaviour::state::WalkState` (whether the walk is stepping), for the hand-off.
- `message::booster::NUSimBallSource`, `NUSimBallCrossings` (NUSim only): with the source at `TRUE_CROSSING`, the crossings of the goalie's line and our goal line come from NUSim rolling the ball ahead without the robot, not from the prediction, with zero σ. `SavePlan.true_crossing` says so. Without a forecast from the last `ball.timeout`, the ball counts as invalid rather than falling back to the prediction. On a real robot neither message exists.

## Emits

- `message::skill::Block` Task: an inactive command (GUARD) or the live shot command (BLOCK).
- `message::strategy::WalkToFieldPosition` Task: the chosen spot, facing the ball (IDLE).
- `message::planning::SavePlan`: the tick's prediction and decision.

## Dependencies

- Director
- `skill::K1BlockPolicy` and its config
- A ball filter that publishes covariance (`localisation::BallLocalisation` from `tumminello/ball-ukf-fixes` on)

## Results in NUSim

These results are for run 12 and its envelope. The `ckfpijgo` retrain hasn't been benchmarked yet.

`tools::GoalieShotBenchmark` (roles/nusim/goalieshots.role) rolled 40 shots from the mjlab "full" level at the goalie through the full stack, on 22 Sep 2026. With ground-truth field localisation, it saved 28 of 40 on-target shots (70%, against 68% in mjlab for run 12), with no falls. PlanSave blocked all 40, a median 0.23 s after the kick (10–90%: 0.17–0.31 s). At the commit, `dy` was off by 0.05 m RMS, against a σ of 0.93 m. So the envelope's expected saves (17 of 40) are conservative, mostly from the too-wide σ.

## Not done yet

- **The envelope only covers starting from the ready stance.** A shot that arrives while the goalie is walking also pays for the mode switch and hand-off, which nothing has measured yet. `guard_distance` is the stopgap.
- **Positioning hasn't been tried through the stack yet.** It is unit-tested against the prototype, but not yet run in NUSim. Its guesses at the real goalie (`extra_lat`, `reach_scale`) are guesses until the goalie is measured on the robot.
- **Positioning ignores the walk and anything but one shot.** It doesn't count the time to get to the spot, or being mid-stride at the kick, and nothing but the penalty area line stops it going out against a dribble or a pass (`line_penalty` is there for that).
- **σ(dy) is ~20× wider than the actual error at the commit.** That delays the commit (P(on target) needs time to reach `min_p_on_target`) and makes expected success conservative. The width comes from the ball filter's kick handling.
- **Vision sees balls on the goalie's own feet and hands in NUSim**, and the ball filter can lock onto one and miss a real shot. `guard_min_ahead` stops PlanSave guarding them, but the filter lock is a vision and localisation fix.
- **No penalty-kick handling.** Nothing here moves before the ball is touched, but PlanSave doesn't know the game state.
