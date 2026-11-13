# FieldLocalisationSRIF

## Description

Estimates where the field is in world space, as a Gaussian in square-root information form. The process
model is rigid-body kinematics driven by velocity states, and every sensor (vision, gyroscope,
accelerometer, odometry, kinematic height) is a measurement with its own noise. Each measurement applies a
MAP (trust-region Newton) update, and the Laplace approximation at the optimum gives the posterior
square-root information.

### State

The state has 18 elements.

|  Index | Symbol    | Meaning                                                       |
| -----: | --------- | ------------------------------------------------------------- |
|   0..2 | `rBFf`    | Torso position in the field frame [m]                         |
|   3..6 | `q`       | Torso attitude quaternion `(w, x, y, z)`, `Rfb = quat2rot(q)` |
|   7..9 | `vBb`     | Body-fixed linear velocity [m/s]                              |
| 10..12 | `omegaBb` | Body-fixed angular velocity [rad/s]                           |
| 13..15 | `bGyro`   | Gyroscope bias [rad/s]                                        |
| 16..17 | `deltaC`  | Camera-mount attitude bias (roll, pitch) [rad]                |

Field frame `{f}`: origin at the centre of the field on the ground plane, z up, own goal at +x.

The attitude is a quaternion, as a roll-pitch-yaw state is singular at pitch = ±90°, which a falling robot
passes through. `quat2rot` normalises, `MeasurementQuaternionNorm` constrains `|q|`, and the mean is
projected back onto the unit sphere after every predict and update. Heading is not a state element, read it
through `SystemLocalisation::heading()`.

The camera bias is a constant error in the torso-to-camera chain, applied on the camera side of the
extrinsic: `Tfc = Tfb(x) · Tbc · R(deltaC)`.

### Initialisation

The first vision frame with enough landmarks triggers a grid search over `(x, y, yaw)`, scored by the
landmark likelihood. Roll, pitch and torso height come from the kinematic chain.

The field is symmetric under a 180° rotation about its centre, so the best pose and its mirror score the
same. The initial pose is taken to be in the own half (+x), which is only valid at kick-off, see
[Limitations](#limitations).

### Measurement updates

| Source                 | Model                        | When                                                 |
| ---------------------- | ---------------------------- | ---------------------------------------------------- |
| YOLO field landmarks   | `MeasurementFieldLandmarks`  | Every frame with usable detections                   |
| Accelerometer          | `MeasurementGravity`         | While the torso is not accelerating                  |
| Kinematic torso height | `MeasurementKinematicHeight` | While upright                                        |
| Unit-norm constraint   | `MeasurementQuaternionNorm`  | Every update                                         |
| Gyroscope              | `MeasurementGyroscope`       | Every frame                                          |
| Body linear velocity   | `MeasurementBodyVelocity`    | Every frame (zero-velocity update while not upright) |

**Body velocity.** `odometry_velocity_source` picks the signal. `HTW_DIFFERENCE` finite-differences
consecutive `Sensors.Htw`. `SENSORS_VTW` rotates `Sensors.vTw` into the torso frame, for platforms such as
the K1 where `vTw` is the controller's odometry twist.

**Landmarks.** Goal posts and L/T/X field-line intersections are unit rays in the camera frame: the box
centre for intersections, the bottom-centre for posts. Detections below `min_confidence` are dropped. Above
it, the confidence sets the inlier weight of a mixture of a Gaussian and uniform clutter. Rays are
associated to landmarks by greedy nearest surprisal, inside a gate that widens with the yaw uncertainty.

**Gravity.** Used while `| ‖a‖ − g | < gravity_quasi_static_tolerance`, whatever the posture.

### Falls

Posture comes from `message::behaviour::state::Stability` (not upright at or below `FALLING`). While not
upright:

- The odometry velocity is replaced by a zero-velocity update, with `zupt_sigma` while `FALLEN` and
  `zupt_dynamic_sigma` while `FALLING` or getting up.
- The kinematic height measurement is not used.
- The `sigma_*_disturbed` process noise applies for the first `process.disturbed_window` seconds.

On standing up, the covariance is widened by `recovery_pos_std` and `recovery_yaw_std` without moving the
mean, which also reopens the landmark association gate. Frames with no usable detections still run a
prediction.

### Output

The emitted `Hfw` is planar, `Translation(x, y, 0) * Rz(yaw)`, as for `FieldLocalisationNLopt`. The
covariance is of `(x, y, yaw)`.

## Usage

Add `localisation::FieldLocalisationSRIF` to a role. It needs a `FieldDescription`, `Sensors` and
`BoundingBoxes`. Without `Stability` the robot is treated as always upright. Emit `ResetFieldLocalisation`
to drop the estimate and re-run the initial grid search.

`data/config/FieldLocalisationSRIF.yaml` holds the parameters that change with the robot, the venue or the
game. The rest are constants in `struct Config` in `FieldLocalisationSRIF.hpp`,
`SystemLocalisation::Parameters` and `MeasurementFieldLandmarks::Options`.

| Key                          | Meaning                                             |
| ---------------------------- | --------------------------------------------------- |
| `use_hypothesis_bank`        | Multi-hypothesis mixture over the field symmetry    |
| `use_side_disambiguator`     | Out-of-field side disambiguation                    |
| `use_gravity`                | Accelerometer gravity direction measurement         |
| `use_kinematic_height`       | Torso height from the support-leg chain             |
| `gyroscope_sigma`            | Gyroscope noise [rad/s]                             |
| `odometry_velocity_source`   | `HTW_DIFFERENCE` or `SENSORS_VTW`                   |
| `odometry_velocity_sigma`    | Body linear velocity noise [m/s]                    |
| `gravity_sigma`              | Accelerometer gravity direction noise [m/s²]        |
| `height_sigma`               | Torso height noise [m]                              |
| `measurement.sigma_angular`  | Inlier ray angular noise [rad]                      |
| `measurement.gate_angle`     | Association gate [rad], widened by the yaw std      |
| `measurement.min_confidence` | YOLO confidence below which a detection is dropped  |
| `process.sigma_vel`          | Body linear velocity random walk [m/s/√s]           |
| `process.sigma_omega`        | Body angular velocity random walk [rad/s/√s]        |
| `fall.recovery_pos_std`      | Horizontal position std restored on standing up [m] |
| `fall.recovery_yaw_std`      | Yaw std restored on standing up [rad]               |

## Consumes

- `message::support::FieldDescription` to build the landmark map
- `message::input::Sensors` for `Htw`, `vTw`, the accelerometer and the gyroscope
- `message::vision::BoundingBoxes` for YOLO detections as corner unit rays in `{c}`, and `Hcw`
- `message::input::Image` for out-of-field corner detection (`use_side_disambiguator` only)
- `message::behaviour::state::Stability` (optional) for the posture
- `message::localisation::ResetFieldLocalisation` to re-initialise

## Emits

- `message::localisation::Field` with the planar `Hfw`, the `(x, y, yaw)` covariance, `uncertainty` (its
  trace), hypothesis `particles`, and `cost` (mean chordal angular residual of the associated rays [rad])
- `message::vision::OutOfFieldFeatures` with every detected corner and predicted landmark as a ray in `{c}`
  and its association outcome, for display
- NUsight graphs of the pose, uncertainty, cost, association count and side evidence, at `DEBUG` log level

NUsight's localisation view draws the `(x, y)` covariance as an uncertainty ellipse on the field.

## Dependencies

- `utility::gaussian_filtering` for the square-root information Gaussian, the event, measurement and system
  base classes, and the trust-region optimiser
- Eigen
- autodiff for the gradients and Hessians of the measurement log-likelihoods
- OpenCV for the corner features of the side disambiguator

## Limitations

- On-field landmarks fit a pose and its mirror equally well, so only `srif::SideDisambiguator`
  (`use_side_disambiguator`, off by default) can correct a wrong-side estimate after kick-off. It maps
  out-of-field corner features and scores them against the pose and its mirror. With the hypothesis bank on,
  its evidence goes into the mixture weights. With it off, a sustained mirror preference flips the belief.
- The out-of-field map is only valid if it was built while the estimate was on the correct side, so map
  building freezes when the evidence favours the mirror.
- Only the torso pose is localised.
- `MeasurementFieldLines` is not used.
- The zero-velocity update assumes the robot is not travelling while not upright, which is wrong if it is
  picked up while it reads `FALLEN`.
