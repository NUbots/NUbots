# FieldLocalisationSRIF

## Description

Estimates where the field is in world space as a Gaussian in square-root information form. The process
model is rigid-body kinematics driven by velocity states; vision, gyroscope, accelerometer, odometry
and kinematic height all enter as measurements. Each vision frame applies a MAP (trust-region Newton)
update over a robust landmark likelihood, and the Laplace approximation at the optimum gives the
posterior, so every pose comes with a covariance.

### State

The state is 18 elements: the 6-DOF torso pose in the field frame `{f}`, 6-DOF body-fixed velocities, a
3-DOF gyroscope bias and a 2-DOF camera-mount attitude bias.

|  Index | Symbol   | Meaning                                                       |
| -----: | -------- | ------------------------------------------------------------- |
|   0..2 | `rTFf`   | Torso position in the field frame [m]                         |
|   3..6 | `q`      | Torso attitude quaternion `(w, x, y, z)`, `Rfb = quat2rot(q)` |
|   7..9 | `v`      | Torso fixed linear velocities [m/s]                           |
| 10..12 | `omega`  | Torso fixed rotational velocities [rads/s]                    |
| 13..15 | `bG`     | Gyroscope bias estimates [rads/s]                             |
| 16..17 | `deltaC` | Camera-mount attitude bias (roll, pitch) [rad]                |

Field frame `{f}`: origin at the centre of the field on the ground plane, z up.

Attitude is a quaternion because roll-pitch-yaw is singular at pitch = ±90°, which every topple
passes through. The redundant fourth parameter is handled three ways: `quat2rot` normalises, so `|q|`
cannot corrupt the geometry; `MeasurementQuaternionNorm` supplies the only information along `|q|`
(otherwise the MAP Hessian is singular); and the mean is projected back onto the unit sphere after
every predict and update. No index is the heading; use `SystemLocalisation::heading()`.

The camera bias models a constant error in the torso-to-camera chain (seen as ground-projection error
growing with range²). It is a random walk with tiny process noise, applied as
`Tfc = Tfb(x) · Tbc · R(deltaC)`.

### Initialisation

The first vision frame carrying enough landmarks triggers a coarse grid search over `(x, y, yaw)`,
scored by the same robust landmark likelihood used for updates. Roll, pitch and torso height are taken
from the gravity-aligned kinematic chain rather than searched.

The maximum and its 180° mirror score identically. The tie is broken by assuming the robot starts in
its own half, **+x** (own goal at `+field_length/2`, as hardcoded across the codebase). This only
holds at kick-off and fails near the halfway line; see [Limitations](#limitations).

### Measurement updates

| Source                 | Model                        | When                                 |
| ---------------------- | ---------------------------- | ------------------------------------ |
| YOLO field landmarks   | `MeasurementFieldLandmarks`  | Every frame with usable detections   |
| Accelerometer          | `MeasurementGravity`         | While quasi-static (see below)       |
| Kinematic torso height | `MeasurementKinematicHeight` | While upright only                   |
| Unit-norm prior        | `MeasurementQuaternionNorm`  | Every update                         |
| Gyroscope              | `MeasurementGyroscope`       | Every frame, any posture             |
| Body linear velocity   | `MeasurementBodyVelocity`    | Every frame (ZUPT while not upright) |

The gyroscope measures `omegaBb + bGyro`. Estimating the bias matters because the upstream Mahony
filter cannot observe its yaw component, which otherwise becomes heading drift.

**Body velocity.** `odometry_velocity_source` picks the signal: `HTW_DIFFERENCE` finite-differences
consecutive `Sensors.Htw` (any platform); `SENSORS_VTW` rotates `Sensors.vTw` into the torso frame
(the K1, whose controller reports an odometry twist). Sensors discards implausible twists and zeroes
latched ones.

**Odometry discontinuities.** `Sensors.Htw` jumps when Sensors re-zeros the odometry
(`ResetFieldLocalisation`, or a Booster mode change). A world-frame xy or yaw step between consecutive
`Htw` larger than `odometry_jump_xy`/`odometry_jump_angle` marks a discontinuity. That sample's
finite-differenced velocity is discarded, and vision frames within `odometry_jump_holdoff` of it skip
their landmark, init and side-disambiguation updates (their `Hcw` and paired `Htw` may straddle it)
and re-emit `Hfw` against the newest `Htw`. The field-frame belief itself is unaffected.

**Landmarks.** Goal posts and L/T/X field-line intersections arrive as unit rays in the camera frame:
the box centre for intersections, the bottom-centre for posts. Detections below `min_confidence` are
dropped; above it, YOLO confidence scales the **inlier weight** of a robust mixture rather than the
noise sigma, since a weak detection's failure mode is not being a landmark at all. Association is greedy surprisal-nearest-neighbour against rays predicted at the prior
mean, inside a geometric pre-gate that widens with the filter's own yaw uncertainty.

**Gravity.** Gated on `| ‖a‖ − g | < gravity_quasi_static_tolerance`, not on posture: it holds for a
robot lying still and fails in free fall or on impact.

### Falls

Each measurement is gated on its own assumption. Only kinematic height is invalidated by a fall (lying
down, the leg chain still reports an upright torso); landmarks and the gyroscope stay valid and catch
any spin during the topple or getup.

Posture comes from `message::behaviour::state::Stability` (not upright at or below `FALLING`). While
not upright:

- The odometry velocity is **replaced by a zero-velocity update** for the whole episode, so the
  pre-fall gait does not integrate through the fall: `zupt_sigma` while `FALLEN`, the looser
  `zupt_dynamic_sigma` while `FALLING` or getting up.
- The `sigma_*_disturbed` PSDs apply for the first `process.disturbed_window` seconds only, so the
  belief's width reflects how far the robot could have moved rather than how long it was down.

On standing up the covariance is widened (mainly yaw, as a rank-one block about field z) without
moving the mean, which also reopens the association gate. Frames without detections still predict.

### Output

The emitted `Hfw` is **planar** (`x`, `y`, yaw), as in `FieldLocalisationNLopt`. The `(x, y, yaw)`
covariance takes yaw through row 2 of `SystemLocalisation::attitude_jacobian`.

## Usage

Add to a role:

```
localisation::FieldLocalisationSRIF
```

Requires a `FieldDescription` (the landmark map is built from it at startup), a `Sensors` stream for
odometry, and `BoundingBoxes` from the vision pipeline. `Stability` is optional — without it the robot
is treated as always upright and the fall handling never engages.

Emit `ResetFieldLocalisation` to drop the estimate and re-run the initial grid search.

`data/config/FieldLocalisationSRIF.yaml` holds the per-robot, venue and game parameters listed below.
Everything else is a constant in `struct Config` (`FieldLocalisationSRIF.hpp`) and
`SystemLocalisation::Parameters`.

The yaml values were tuned in NUSim (see [Tuning](#tuning)).

**What the filter runs**

| Key                      | Meaning                                                                   |
| ------------------------ | ------------------------------------------------------------------------- |
| `use_hypothesis_bank`    | Multi-hypothesis mixture; off by default, see Limitations                 |
| `use_side_disambiguator` | Out-of-field side disambiguation; the only thing that resolves the mirror |
| `use_gravity`            | Accelerometer gravity direction as a secondary attitude anchor            |
| `use_kinematic_height`   | Torso height from the support-leg chain                                   |

**How far each sensor is trusted** — noise std devs; bigger means weighed less

| Key                        | Meaning                                                                |
| -------------------------- | ---------------------------------------------------------------------- |
| `gyroscope_sigma`          | Gyroscope noise [rad/s]; drives the body angular velocity and its bias |
| `odometry_velocity_source` | `HTW_DIFFERENCE` or `SENSORS_VTW` — which signal measures `vBb`        |
| `odometry_velocity_sigma`  | Body linear velocity measurement [m/s]; size it for the source chosen  |
| `gravity_sigma`            | Accelerometer gravity direction [m/s²]                                 |
| `height_sigma`             | Torso height from the support-leg chain [m]                            |

**How far vision is trusted** (`measurement:`)

| Key                      | Meaning                                                                      |
| ------------------------ | ---------------------------------------------------------------------------- |
| `sigma_angular`          | Inlier ray angular noise [rad]; total per-frame error, not just px           |
| `gate_angle`             | Nominal association pre-gate [rad]; widened at runtime by yaw std            |
| `min_confidence`         | YOLO confidence below which a detection is discarded outright                |
| `gate_yaw_scale`         | Yaw std devs the pre-gate widens by, so an uncertain belief can re-associate |
| `gate_angle_max`         | Ceiling on the widened pre-gate [rad]                                        |
| `inlier_probability`     | Robust mixture inlier weight of a detection at `confidence_reference`        |
| `confidence_reference`   | YOLO confidence that maps to `inlier_probability`                            |
| `max_inlier_probability` | Cap on the inlier weight, so no detection is treated as certainly real       |

**How fast the belief may move** (`process:`) — the dominant process noise

| Key               | Meaning                                                                       |
| ----------------- | ----------------------------------------------------------------------------- |
| `sigma_vel`       | Body linear velocity random walk [m/s/√s]; raise if the filter lags the robot |
| `sigma_omega`     | Body angular velocity random walk [rad/s/√s]; lower if the pose is twitchy    |
| `sigma_pos_xy`    | Horizontal position random walk [m/√s], a floor on top of the velocity states |
| `sigma_yaw`       | Yaw random walk [rad/√s], a floor on top of the velocity states               |
| `sigma_gyro_bias` | Gyroscope bias random walk [rad/s/√s]                                         |
| `sigma_cam_bias`  | Camera-mount attitude bias random walk [rad/√s]                               |

**How much confidence a fall costs** (`fall:`)

| Key                | Meaning                                                                  |
| ------------------ | ------------------------------------------------------------------------ |
| `recovery_pos_std` | Horizontal position std restored on standing up [m]                      |
| `recovery_yaw_std` | Yaw std restored on standing up [rad]; also reopens the association gate |

### Tuning

The yaml values were tuned by replaying NUSim recordings offline against ground truth
(`message::localisation::RobotPoseGroundTruth`). NUSim's vision is cleaner than a real camera's, so
`sigma_angular` and the association gate may need loosening on a real robot. Recordings carry no
`Stability`, so the `fall:` values were not tuned this way.

## Consumes

- `message::support::FieldDescription` — field dimensions; the landmark map is built from this at startup
- `message::input::Sensors` — `Htw` odometry, `accelerometer`, `gyroscope`
- `message::vision::BoundingBoxes` — YOLO detections as corner unit rays in `{c}`, plus `Hcw`
- `message::input::Image` — the raw camera frame (RGB3, BGR3, RGBA, RGGB or GREY), for out-of-field
  corner detection (`use_side_disambiguator` only)
- `message::behaviour::state::Stability` — optional; posture for the fall gate
- `message::localisation::ResetFieldLocalisation` — forces re-initialisation

## Emits

- `message::localisation::Field` — planar `Hfw`, `(x, y, yaw)` covariance, `uncertainty` (its trace),
  hypothesis `particles`, `cost` (mean chordal angular residual of the associated rays [rad]), and
  `localised`, set while the position and yaw std are under `Config::localised_pos_std` and
  `localised_yaw_std`
- `message::vision::OutOfFieldFeatures` — per-frame side-disambiguator state for the NUsight vision
  pane (corners, predicted landmarks, LLR). Display only.
- NUsight graphs for pose, uncertainty, cost, association count and the side evidence, at `DEBUG` log
  level

NUsight's localisation view draws the `(x, y)` covariance as a 3σ ellipse (**Debug → Uncertainty**).
A pose drifting while the ellipse stays small means the filter is confidently wrong.

The vision pane draws out-of-field corners over the image (**Out of Field**): cyan unclaimed
background, steel candidate track, green matched at the current pose, **yellow matched only at the
mirrored pose** (wrong-side evidence, precedes a flip), red rejected, faint grey masked as carpet.
Rings are predicted landmarks (thin when bearing-only), joined to their matched corner by the
residual.

## Dependencies

- `utility::gaussian_filtering` (`shared/utility/gaussian_filtering`) — the generic estimator scaffolding, nothing localisation
  specific: `GaussianInfo` (square-root information Gaussian), the `Event`/`Measurement` and
  `SystemBase`/`SystemEstimator` base classes, `Pose`, the rotation and kinematics helpers, and the
  trust-region optimiser (`funcmin`)
- `src/srif` and `src/measurement` (this module) — everything specific to this filter: the state
  layout (`SystemLocalisation`), `FieldMap`, the log-replay sample types, the side disambiguator and
  the concrete measurement models
- Eigen, and autodiff for the gradients and Hessians of the measurement log-likelihoods

## Limitations

- **Only out-of-field evidence resolves the 180° symmetry** after initialisation; on-field landmarks
  (and the hypothesis bank alone) cannot. `srif::SideDisambiguator` maps out-of-field FAST/ORB corners
  online (mostly bearing-only) and scores the pose against its mirror (~4 ms/frame, on its own
  reaction). With the hypothesis bank on, the evidence reweights the mixture
  (`add_side_log_evidence`); with it off, sustained mirror preference flips the belief. With
  `use_side_disambiguator` off (the default), only the own-half rule at initialisation breaks the
  symmetry, so a reset near the halfway line can lock onto the mirror.
- The out-of-field map is only valid if built on the correct side; building freezes when the evidence
  favours the mirror, and the map is cleared on `ResetFieldLocalisation`.
- Only the torso pose is localised; feet in `{f}` are not emitted.
- `MeasurementFieldLines` exists but is not wired in.
- The zero-velocity update is wrong if a handler carries the robot while it reads `FALLEN`
  (`Config::zupt_sigma`).
