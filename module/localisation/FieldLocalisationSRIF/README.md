# FieldLocalisationSRIF

## Description

A localisation method for estimating where the field is in world space, as a Gaussian in square-root
information form. Nothing is a known input: the process model is rigid-body kinematics driven by
velocity states, and every sensor — vision, gyroscope, accelerometer, walk odometry, kinematic height
— enters as a measurement with its own noise. Each vision frame applies a MAP (trust-region Newton)
update over a robust landmark likelihood, and the Laplace approximation at the optimum gives the
posterior square-root information. Every estimate therefore carries a covariance saying how much to
trust it, rather than a bare pose.

### State

The state is nine elements: the 6-DOF torso pose in the field frame `{f}` plus a 2-DOF camera-mount
attitude bias.

| Index | Symbol   | Meaning                                                       |
| ----: | -------- | ------------------------------------------------------------- |
|  0..2 | `rBFf`   | Torso position in the field frame [m]                         |
|  3..6 | `q`      | Torso attitude quaternion `(w, x, y, z)`, `Rfb = quat2rot(q)` |
|  7..8 | `deltaC` | Camera-mount attitude bias (roll, pitch) [rad]                |

Field frame `{f}`: origin at the centre of the field on the ground plane, z up, matching the NUbots
`Hfw` convention.

Attitude is a quaternion rather than roll-pitch-yaw because the Euler rate transform is singular at
pitch = ±90°, which is on the trajectory of every topple. Passing through it put the old state on the
alias `(roll+180, 180−pitch, yaw+180)` — the same rotation, so the geometry kept working, but every
consumer reading the yaw element was then 180° out. A quaternion has no such point, so the filter can
sit face-down and keep updating. The cost is a fourth parameter for three degrees of freedom, handled
three ways: `quat2rot` normalises, so `|q|` is invisible to every geometric model and cannot corrupt
the attitude; `MeasurementQuaternionNorm` supplies the only information along that direction (without
it the MAP Hessian is singular there); and the mean is projected back onto the unit sphere after every
predict and update. No index means "heading" any more — read it through
`SystemLocalisation::heading()`.

The camera bias models a constant error in the kinematic torso-to-camera chain, visible in recorded
data as ground-projection error growing with range² (~1 m at 4–5 m range, consistent with a 1.5–2°
pitch bias). It is a random-walk state with deliberately tiny process noise, applied on the camera
side of the extrinsic: `Tfc = Tfb(x) · Tbc · R(deltaC)`.

### Initialisation

The first vision frame carrying enough landmarks triggers a coarse grid search over `(x, y, yaw)`,
scored by the same robust landmark likelihood used for updates. Roll, pitch and torso height are taken
from the gravity-aligned kinematic chain rather than searched.

The field is symmetric under a 180° rotation about its centre, so the maximum and its mirror score
identically. The tie is broken by game context: `own_half_x_sign` picks the half the robot starts in.
That prior is only true at kick-off — see [Limitations](#limitations).

### Measurement updates

| Source                 | Model                        | When                               |
| ---------------------- | ---------------------------- | ---------------------------------- |
| YOLO field landmarks   | `MeasurementFieldLandmarks`  | Every frame with usable detections |
| Accelerometer          | `MeasurementGravity`         | While quasi-static (see below)     |
| Kinematic torso height | `MeasurementKinematicHeight` | While upright only                 |
| Unit-norm prior        | `MeasurementQuaternionNorm`  | Every update                       |

The gyroscope is **not** a measurement update — it enters as the angular velocity of the body-twist
prediction input, bias-calibrated on quiet samples. The walk-engine odometry attitude slips badly
while turning; the gyro does not.

**Landmarks.** Goal posts and L/T/X field-line intersections arrive as unit rays in the camera frame:
the box centre for intersections, the bottom-centre for posts. Detections below `min_confidence` are
dropped outright; above it, YOLO confidence scales the **inlier weight** of a robust mixture rather
than the noise sigma. That is deliberate — scaling sigma would claim the landmark is certainly real
but poorly measured, whereas the actual failure mode of a weak detection is that it is not a landmark
at all. As the weight tends to zero the per-detection likelihood tends to the flat clutter term, which
contributes almost nothing to either the gradient or the Hessian, so a weak detection cannot sharpen
the posterior. Association is greedy surprisal-nearest-neighbour against rays predicted at the prior
mean, inside a geometric pre-gate that widens with the filter's own yaw uncertainty.

**Gravity.** Gated on `| ‖a‖ − g | < gravity_quasi_static_tolerance`, which is the model's real
validity condition: the accelerometer reads gravity whenever the torso is not being accelerated. That
is true of a robot lying still on the carpet and false of one in free fall or hitting the ground, so
it is both the right test during a fall and a better one than "is the robot upright" while walking.

### Falls

The gate is per-model, not all-or-nothing. Only kinematic height is genuinely invalidated by a fall —
lying down, the support-leg chain still reports a near-upright 0.44 m torso and would fight the
attitude the other measurements are establishing. Landmarks are plain geometry and, given the right
attitude, are as valid face-down as standing. A robot that spins while toppling or getting up changes
its heading, and only measurements taken _during_ the event can catch that.

Posture comes from `message::behaviour::state::Stability` (not upright at or below `FALLING`). While
not upright, two separate things happen, and they expire differently:

- The walk-odometry velocity measurement is **replaced by a zero-velocity update** for the whole
  episode. On the ground the odometry describes a gait that is not happening, and during a getup a
  scripted flail that is not locomotion. Leaving `vBb` unmeasured instead is not the neutral choice it
  looks like — it asserts the robot may still be travelling at whatever it was doing when it fell,
  which is the one thing it certainly is not doing, and the pre-fall velocity then integrates for the
  whole fall. Measured on data4_webots, that failure compounds the error 0.12 → 0.62 → 1.76 m across
  two falls and never recovers. The sigma varies with posture: `zupt_sigma` while `FALLEN` (lying
  still), `zupt_dynamic_sigma` while `FALLING` or getting up, where the torso genuinely moves — just
  not anywhere. The gyroscope measurement runs throughout, because it measures the topple for real.
- The elevated `sigma_*_disturbed` PSDs apply for `process.disturbed_window` seconds and then stand
  down. This one _is_ bounded: a fall is a bounded event, and modelling a robot lying still as a
  0.40 m/√s random walk would make the belief's width report how long it had been down rather than how
  far it could have gone.

Keeping those as one switch is a trap: it means that from `disturbed_window` seconds into a fall the
filter resumes integrating getup odometry at walking-grade confidence, and marches off the field with
a covariance too tight for the association gate to recover it.

On standing up, the belief is widened without moving its mean. A fall and getup translate the torso
well under a metre, so the pre-fall position is still the best estimate available, and re-solving
globally would be worse because the own-half prior is false once play is under way. What a fall
destroys is confidence, above all in yaw, so that is what is handed back — as a rank-one block about
the field z axis, since yaw is a direction in the quaternion block rather than an element. The widened
belief is also what reopens the landmark association gate; without it a getup that turned the robot
leaves every predicted bearing outside the gate and the filter can never re-acquire.

Frames with no usable detections still run a prediction, so the belief decays honestly across a fall
instead of emerging with pre-fall confidence in a mean that has moved.

### Output

The emitted `Hfw` is **planar**: `(x, y)` and yaw, with torso height and roll/pitch dropped. The field
is a flat z = 0 model, so a full SE(3) `Hfw` (carrying the walking torso's tilt and ~0.4 m height)
would tip the field lines off the plane in NUsight. This matches the `FieldLocalisationNLopt`
convention, and every consumer reasons about the field on the ground plane anyway.

The reported `(x, y, yaw)` covariance takes yaw through row 2 of
`SystemLocalisation::attitudeJacobian`, since yaw is a direction in the quaternion block rather than a
state element — that applies to its cross-covariance with position too.

## Usage

Add to a role:

```
localisation::FieldLocalisationSRIF
```

Already enabled in `roles/webots/localisation.role` and `roles/test/localisation.role`.

Requires a `FieldDescription` (the landmark map is built from it at startup), a `Sensors` stream for
odometry, and `BoundingBoxes` from the vision pipeline. `Stability` is optional — without it the robot
is treated as always upright and the fall handling never engages.

Emit `ResetFieldLocalisation` to drop the estimate and re-run the initial grid search.

Tuning lives in `data/config/FieldLocalisationSRIF.yaml`. The values most worth knowing about:

| Key                                          | Meaning                                                                 |
| -------------------------------------------- | ----------------------------------------------------------------------- |
| `own_half_x_sign`                            | Sign of field-x for the starting half; breaks the 180° symmetry at init |
| `measurement.min_confidence`                 | YOLO confidence below which a detection is discarded                    |
| `measurement.gate_angle`                     | Nominal association pre-gate [rad]                                      |
| `measurement.gate_yaw_scale`                 | How many yaw std devs the pre-gate widens to when uncertain             |
| `process.sigma_*` / `sigma_*_disturbed`      | Process noise PSDs, upright and while down                              |
| `process.disturbed_window`                   | How long the disturbed PSDs apply [s] (bounds the PSDs only)            |
| `fall.recovery_pos_std` / `recovery_yaw_std` | Confidence handed back on standing up                                   |
| `gravity_quasi_static_tolerance`             | How far ‖a‖ may sit from g and still count as gravity [m/s²]            |
| `use_hypothesis_bank`                        | Multi-hypothesis mixture; off by default, see Limitations               |

## Consumes

- `message::support::FieldDescription` — field dimensions; the landmark map is built from this at startup
- `message::input::Sensors` — `Htw` odometry, `accelerometer`, `gyroscope`
- `message::vision::BoundingBoxes` — YOLO detections as corner unit rays in `{c}`, plus `Hcw`
- `message::behaviour::state::Stability` — optional; posture for the fall gate
- `message::localisation::ResetFieldLocalisation` — forces re-initialisation

## Emits

- `message::localisation::Field` — planar `Hfw`, `(x, y, yaw)` covariance, `uncertainty` (its trace),
  hypothesis `particles`, and `cost` (mean chordal angular residual of the associated rays [rad])
- NUsight graphs for pose, uncertainty, cost and association count, at `DEBUG` log level

## Dependencies

- `utility::slam` (`shared/utility/slam`) — the estimator core: `GaussianInfo` (square-root
  information Gaussian), `SystemLocalisation`, the measurement models, `FieldMap`, and the
  trust-region optimiser
- Eigen, and autodiff for the gradients and Hessians of the measurement log-likelihoods

## Limitations

- **The 180° field symmetry is only broken at initialisation.** On-field landmarks fit a pose and its
  mirror equally well, so nothing downstream recovers a wrong-side lock. `use_hypothesis_bank` carries
  a mirror hypothesis, but each component is scored on its own association and the pair sits at a
  genuine 50/50, so the bank alone cannot resolve it — hence off by default. Breaking the tie needs
  asymmetric off-field evidence fed in through `SystemLocalisation::addSideLogEvidence`.
  `utility::slam::SideDisambiguator` implements exactly that — out-of-field FAST/ORB corner landmarks
  classified geometrically against the carpet and horizon, triangulated online (mostly as bearing-only
  landmarks, since distant background rarely accrues usable parallax), then scored against the pose and
  its mirror — but **this module does not source it yet**. Future work; it costs roughly 4 ms/frame.
- There is no recovery path for a mid-game kidnap: re-initialisation would re-apply the own-half prior,
  which is false once play is under way.
- Only the torso pose is localised. Foot poses in the field frame would come from composing the
  kinematic foot frames (`Sensors.Htx[L_FOOT_BASE]`/`[R_FOOT_BASE]`) with the field pose; not emitted.
- `MeasurementFieldLines` exists in `utility::slam` but is not wired in — this module localises from
  YOLO landmarks alone, not raw field-line points.
- The zero-velocity update asserts the robot is not travelling for the whole non-upright window. That
  is sound for a topple and a getup, but wrong if a handler picks the robot up while it still reads
  `FALLEN`; `zupt_sigma` is the knob if that becomes a problem in a real game.
