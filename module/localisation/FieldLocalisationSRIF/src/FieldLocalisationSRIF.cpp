/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "FieldLocalisationSRIF.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/BoundingBoxes.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/slam/FieldMapFromDescription.hpp"
#include "utility/slam/gaussian/GaussianInfo.hpp"
#include "utility/slam/measurement/MeasurementGravity.hpp"
#include "utility/slam/measurement/MeasurementKinematicHeight.hpp"
#include "utility/slam/measurement/MeasurementQuaternionNorm.hpp"
#include "utility/slam/rotation.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::localisation {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::localisation::ResetFieldLocalisation;
    using message::support::FieldDescription;
    using message::vision::BoundingBox;
    using message::vision::BoundingBoxes;

    using utility::nusight::graph;
    using utility::slam::rot2rpy;
    using utility::slam::rpy2quat;
    using utility::slam::gaussian::GaussianInfo;
    using utility::slam::measurement::MeasurementGravity;
    using utility::slam::measurement::MeasurementKinematicHeight;
    using utility::slam::measurement::MeasurementQuaternionNorm;
    using utility::support::Expression;

    /// @brief Convert an Eigen isometry to the filter's pose type
    static filter::Pose<double> to_pose(const Eigen::Isometry3d& H) {
        filter::Pose<double> T;
        T.rotationMatrix    = H.rotation();
        T.translationVector = H.translation();
        return T;
    }

    /// @brief Convert the filter's pose type to an Eigen isometry
    static Eigen::Isometry3d to_isometry(const filter::Pose<double>& T) {
        Eigen::Isometry3d H = Eigen::Isometry3d::Identity();
        H.linear()          = T.rotationMatrix;
        H.translation()     = T.translationVector;
        return H;
    }

    /// @brief Build a filter VisionSample (rays in camera frame {c}) from YOLO bounding boxes.
    ///
    /// Each BoundingBox carries the class name, the YOLO confidence, and the four corner unit rays in
    /// {c} (ordered TL, TR, BR, BL - matching MeasurementFieldLandmarks::detectionRay, which takes the
    /// box centre for intersections and the bottom-centre for goal posts). detectionRay also filters to
    /// the mapped landmark classes and drops low-confidence detections, so every usable box is passed
    /// through here and the confidence drives the robust inlier weight.
    static filter::VisionSample build_vision_sample(double t, const BoundingBoxes& boxes) {
        filter::VisionSample sample;
        sample.t          = t;
        sample.videoFrame = -1;
        sample.Hcw        = to_pose(Eigen::Isometry3d(boxes.Hcw));

        for (const BoundingBox& box : boxes.bounding_boxes) {
            if (box.corners.size() != 4) {
                continue;  // detectionRay expects the four box-corner rays
            }
            filter::Detection det;
            det.name       = box.name;
            det.confidence = box.confidence;
            for (int c = 0; c < 4; ++c) {
                det.corners.col(c) = box.corners[std::size_t(c)];
            }
            sample.detections.push_back(std::move(det));
        }

        return sample;
    }

    FieldLocalisationSRIF::FieldLocalisationSRIF(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("FieldLocalisationSRIF.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.own_half_x_sign        = config["own_half_x_sign"].as<double>();
            cfg.use_hypothesis_bank    = config["use_hypothesis_bank"].as<bool>();
            cfg.use_gyroscope          = config["use_gyroscope"].as<bool>();
            cfg.use_gravity            = config["use_gravity"].as<bool>();
            cfg.gravity_sigma          = config["gravity_sigma"].as<double>();
            cfg.use_kinematic_height   = config["use_kinematic_height"].as<bool>();
            cfg.height_sigma           = config["height_sigma"].as<double>();
            cfg.max_odometry_gap       = config["max_odometry_gap"].as<double>();
            cfg.twist_window_seconds   = config["twist_window_seconds"].as<double>();
            cfg.max_sensor_pairing_age = config["max_sensor_pairing_age"].as<double>();

            cfg.gravity_quasi_static_tolerance = config["gravity_quasi_static_tolerance"].as<double>();
            cfg.quaternion_norm_sigma          = config["quaternion_norm_sigma"].as<double>();
            cfg.recovery_pos_std               = config["fall"]["recovery_pos_std"].as<double>();
            cfg.recovery_yaw_std               = config["fall"]["recovery_yaw_std"].as<double>();

            cfg.grid_step_xy          = config["grid_step_xy"].as<double>();
            cfg.grid_step_yaw         = config["grid_step_yaw"].as<double>() * M_PI / 180.0;  // deg -> rad
            cfg.min_init_associations = config["min_init_associations"].as<int>();

            const auto init_sqrt = config["initial_sqrt_covariance"].as<std::vector<double>>();
            for (Eigen::Index i = 0; i < cfg.initial_sqrt_covariance.size() && i < Eigen::Index(init_sqrt.size());
                 ++i) {
                cfg.initial_sqrt_covariance(i) = init_sqrt[std::size_t(i)];
            }

            // Process noise PSDs
            cfg.process.sigmaPosXY   = config["process"]["sigma_pos_xy"].as<double>();
            cfg.process.sigmaPosZ    = config["process"]["sigma_pos_z"].as<double>();
            cfg.process.sigmaAtt     = config["process"]["sigma_att"].as<double>();
            cfg.process.sigmaYaw     = config["process"]["sigma_yaw"].as<double>();
            cfg.process.sigmaCamBias = config["process"]["sigma_cam_bias"].as<double>();

            // Process noise PSDs while the robot is not upright
            cfg.process.sigmaPosXYDisturbed = config["process"]["sigma_pos_xy_disturbed"].as<double>();
            cfg.process.sigmaPosZDisturbed  = config["process"]["sigma_pos_z_disturbed"].as<double>();
            cfg.process.sigmaAttDisturbed   = config["process"]["sigma_att_disturbed"].as<double>();
            cfg.process.sigmaYawDisturbed   = config["process"]["sigma_yaw_disturbed"].as<double>();
            cfg.process.disturbedWindow     = config["process"]["disturbed_window"].as<double>();

            // Landmark measurement options
            cfg.measurement.sigmaAngular         = config["measurement"]["sigma_angular"].as<double>();
            cfg.measurement.gateAngle            = config["measurement"]["gate_angle"].as<double>();
            cfg.measurement.gateYawScale         = config["measurement"]["gate_yaw_scale"].as<double>();
            cfg.measurement.gateAngleMax         = config["measurement"]["gate_angle_max"].as<double>();
            cfg.measurement.minConfidence        = config["measurement"]["min_confidence"].as<double>();
            cfg.measurement.inlierProbability    = config["measurement"]["inlier_probability"].as<double>();
            cfg.measurement.confidenceReference  = config["measurement"]["confidence_reference"].as<double>();
            cfg.measurement.maxInlierProbability = config["measurement"]["max_inlier_probability"].as<double>();

            // Hypothesis-bank parameters
            cfg.hypothesis.minWeight     = config["hypothesis"]["min_weight"].as<double>();
            cfg.hypothesis.maxComponents = config["hypothesis"]["max_components"].as<std::size_t>();
            cfg.hypothesis.mergePosition = config["hypothesis"]["merge_position"].as<double>();
            cfg.hypothesis.mergeYaw      = config["hypothesis"]["merge_yaw"].as<double>();
            cfg.hypothesis.respawnPosStd = config["hypothesis"]["respawn_pos_std"].as<double>();

            // Propagate live tuning into an already-constructed estimator
            if (system != nullptr) {
                system->params = cfg.process;
                system->hyp    = cfg.hypothesis;
            }
        });

        on<Startup, Trigger<FieldDescription>>().then("Build field landmark map", [this](const FieldDescription& fd) {
            map = std::make_unique<filter::FieldMap>(utility::slam::field_dimensions(fd));
            log<INFO>("Built field landmark map from FieldDescription");
        });

        on<Trigger<ResetFieldLocalisation>, Sync<FieldLocalisationSRIF>>().then([this] {
            log<INFO>("Resetting field localisation");
            system.reset();
            initialised = false;
        });

        // Maintain the rolling odometry window and rebuild the body-twist input buffer the estimator
        // predicts against. Runs ahead of the vision update (Priority::HIGH) and shares its Sync group,
        // so twist_buffer is never mutated while a prediction is reading it.
        on<Trigger<Sensors>, Sync<FieldLocalisationSRIF>, Priority::HIGH>().then([this](const Sensors& sensors) {
            if (!have_t0) {
                t0      = sensors.timestamp;
                have_t0 = true;
            }

            filter::SensorsSample s;
            s.t             = seconds(sensors.timestamp);
            s.Htw           = to_pose(Eigen::Isometry3d(sensors.Htw));
            s.accelerometer = sensors.accelerometer;
            // twistFromOdometry uses the gyro as the body angular velocity when it is finite, otherwise it
            // falls back to the rate finite-differenced from the odometry attitude in Htw. Feed a NaN when
            // use_gyroscope is off to force that odometry-derived fallback.
            s.gyroscope = cfg.use_gyroscope ? sensors.gyroscope
                                            : Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
            sensors_window.push_back(std::move(s));

            // Drop samples older than the configured window (keep at least two to difference across)
            const double cutoff = sensors_window.back().t - cfg.twist_window_seconds;
            auto first_kept     = std::find_if(sensors_window.begin(), sensors_window.end(), [cutoff](const auto& e) {
                return e.t >= cutoff;
            });
            if (first_kept != sensors_window.begin() && std::distance(first_kept, sensors_window.end()) >= 2) {
                sensors_window.erase(sensors_window.begin(), first_kept);
            }

            // Rebuild in place: assignment keeps the vector object address stable, so the pointer the
            // system holds to twist_buffer stays valid.
            twist_buffer = filter::SystemLocalisation::twistFromOdometry(sensors_window, 0.0, cfg.max_odometry_gap);
        });

        on<Trigger<BoundingBoxes>, Optional<With<Stability>>, Sync<FieldLocalisationSRIF>, Single>().then(
            "SRIF field localisation",
            [this](const BoundingBoxes& boxes, const std::shared_ptr<const Stability>& stability) {
                // Prerequisites: map built and odometry seen.
                if (map == nullptr || !have_t0) {
                    return;
                }

                const double t = seconds(boxes.timestamp);

                // Posture gate. This used to suppress every update while the robot was not
                // upright, on the grounds that all four measurement models assume an upright
                // robot. That was only half right. The models that break are the ones whose
                // *assumption* breaks -- kinematic height needs the support leg on the ground,
                // and the accelerometer only reads gravity when the robot is not being
                // accelerated. The landmark model is plain geometry: given the right attitude it
                // is as valid face-down as standing. What actually forced blanket suppression was
                // the roll-pitch-yaw state, which could not represent a fallen robot without
                // passing through gimbal lock; with a quaternion (see SystemLocalisation) it can,
                // so the gate is per-model rather than all-or-nothing. That matters because a
                // fall is exactly when the estimate is most at risk: a robot that spins while
                // toppling or getting up changes its heading, and only measurements taken during
                // the event can catch it.
                const bool upright = stability == nullptr || *stability > Stability::FALLING;

                // Posture transitions are handled here, before any early return, and never inside
                // one. A getup ends in motion blur, so the frame the robot first reads upright
                // again is very often a frame with no usable detections; handling the transition
                // further down meant the recovery inflation was simply skipped on those runs, and
                // the filter came out of the fall holding pre-fall confidence in a mean that had
                // moved. Setting the posture up here also means it is set from THIS frame's fall
                // start rather than the previous episode's.
                if (initialised) {
                    if (!upright && was_upright) {
                        fall_start_t = t;
                        log<INFO>(
                            "Robot not upright; suppressing kinematic height, landmark and "
                            "(quasi-static) gravity updates continue");
                    }
                    else if (upright && !was_upright) {
                        apply_fall_recovery(t);
                    }
                    system->setPosture(upright, upright ? 0.0 : t - fall_start_t);
                    was_upright = upright;
                }

                // Pair the odometry to the vision capture time. The message's Hcw is at capture, so
                // using the latest Htw instead would fold the capture-to-now torso motion into Tbc
                // and offset every reprojection.
                const filter::SensorsSample* paired = nearest_sensors(t);
                if (paired == nullptr) {
                    log<DEBUG>("No odometry sample near the vision frame; skipping");
                    if (initialised) {
                        system->predictAll(t);
                    }
                    return;
                }
                const filter::Pose<double>& Htw = paired->Htw;

                // Camera pose w.r.t. torso from the kinematic chain (odometry world cancels when Htw
                // and Hcw are from the same time): Tbc = Htw * Hcw^{-1}.
                const filter::Pose<double> Tbc = Htw * to_pose(Eigen::Isometry3d(boxes.Hcw)).inverse();

                const filter::VisionSample sample = build_vision_sample(t, boxes);

                // A face-down fall produces no detections at all. Prediction otherwise only ever
                // happens inside Event::process, so such a frame would advance neither the state
                // nor the clock and the filter would emerge from the fall holding its pre-fall
                // mean at its pre-fall covariance -- confidently wrong rather than honestly
                // uncertain.
                if (sample.detections.empty()) {
                    if (initialised) {
                        system->predictAll(t);
                    }
                    return;
                }

                // Bootstrap: solve a coarse initial pose the first time we see usable landmarks.
                if (!initialised) {
                    const filter::Pose<double> Twt = Htw.inverse();
                    Eigen::Matrix<double, 9, 1> eta0;
                    if (!solve_initial_pose(sample, Tbc, Twt, eta0)) {
                        log<DEBUG>("Initial pose solve did not associate enough landmarks yet");
                        return;
                    }

                    Eigen::MatrixXd S0 =
                        Eigen::MatrixXd::Zero(filter::SystemLocalisation::nx, filter::SystemLocalisation::nx);
                    S0.diagonal() = cfg.initial_sqrt_covariance;
                    const auto p0 = GaussianInfo<double>::fromSqrtMoment(Eigen::VectorXd(eta0), S0);

                    system         = std::make_unique<filter::SystemLocalisation>(p0, twist_buffer);
                    system->params = cfg.process;
                    system->hyp    = cfg.hypothesis;
                    system->resetTo(p0, t);
                    if (cfg.use_hypothesis_bank) {
                        system->initialiseHypotheses();
                    }
                    initialised = true;
                    was_upright = upright;
                    log<INFO>("Initialised field pose: x=",
                              eta0(0),
                              "m y=",
                              eta0(1),
                              "m yaw=",
                              filter::SystemLocalisation::heading(Eigen::VectorXd(eta0)) * 180.0 / M_PI,
                              "deg");
                }

                // Predict forward to the vision capture time using the odometry twist buffer.
                system->predict(t);

                // Unit-norm pseudo-measurement. The four attitude states carry three degrees of
                // freedom and quat2rot normalises, so |q| is invisible to every other model here;
                // without this the MAP Hessian is singular along it and the Newton step has a flat
                // direction to wander down. Applied first so the rest of the frame's updates see a
                // belief that is on the sphere.
                MeasurementQuaternionNorm quaternion_norm(t, cfg.quaternion_norm_sigma);
                system->process(quaternion_norm);

                // Landmark measurement update (routed through process() so the hypothesis bank,
                // when active, applies it to every mixture component and reweights them).
                filter::MeasurementFieldLandmarks measurement(t, sample, Tbc, *map, *system, cfg.measurement);
                system->process(measurement);

                // Low-rate corrections from the paired Sensors sample.
                //
                // Gravity is valid whenever the torso is not being accelerated -- true of a robot
                // lying still on the carpet, false of one in free fall or hitting the ground,
                // regardless of posture. Gating on the specific-force magnitude tests that
                // directly, which is both the right condition during a fall and a better one than
                // "upright" while walking.
                if (cfg.use_gravity && paired->accelerometer.allFinite()) {
                    constexpr double standard_gravity = 9.80665;
                    const double a_mag                = paired->accelerometer.norm();
                    if (std::abs(a_mag - standard_gravity) < cfg.gravity_quasi_static_tolerance) {
                        MeasurementGravity gravity(t, paired->accelerometer, cfg.gravity_sigma);
                        system->process(gravity);
                    }
                }
                // Torso height above ground assumes the support leg reaches the ground, so this is
                // the one model a fall genuinely invalidates: lying down, the chain still reports a
                // near-upright 0.44 m torso and would fight the attitude the other measurements
                // are establishing.
                if (cfg.use_kinematic_height && upright) {
                    const double height = Htw.inverse().translationVector.z();
                    if (std::isfinite(height)) {
                        MeasurementKinematicHeight kinematic_height(t, height, cfg.height_sigma);
                        system->process(kinematic_height);
                    }
                }

                // A diverged update (non-finite mean) would otherwise ship a NaN Hfw, which renders
                // the robot at the field origin. Drop back to re-initialisation instead.
                if (!system->density.mean().allFinite()) {
                    log<WARN>("Field localisation state became non-finite; re-initialising");
                    system.reset();
                    initialised = false;
                    return;
                }

                emit_field(Htw, &measurement);
            });
    }

    void FieldLocalisationSRIF::apply_fall_recovery(double t) {
        // The mean is kept: a fall and getup move the torso well under a metre, so the pre-fall
        // position is still the best estimate available, and re-solving the pose globally would be
        // worse -- the grid search resolves the field symmetry from the known starting half, a
        // prior that is simply false once play is under way. What a fall actually destroys is
        // confidence, above all in yaw, so that is what is given back. The widened belief is also
        // what lets the landmark association gate reopen (see Options::gateYawScale); without it a
        // getup that turned the robot leaves every predicted bearing outside the gate and the
        // filter can never re-associate.
        const Eigen::VectorXd xr = system->density.mean();
        Eigen::MatrixXd extra_cov =
            Eigen::MatrixXd::Zero(filter::SystemLocalisation::nx, filter::SystemLocalisation::nx);
        extra_cov(0, 0) = extra_cov(1, 1) = cfg.recovery_pos_std * cfg.recovery_pos_std;
        // Yaw uncertainty is about the field z axis, which on the quaternion states is a rank-one
        // block rather than a single diagonal element -- there is no "the yaw element" any more.
        const Eigen::Vector4d j_yaw = filter::SystemLocalisation::attitudeTangentField(xr).col(2);
        extra_cov.block<4, 4>(filter::SystemLocalisation::iQuat, filter::SystemLocalisation::iQuat) =
            cfg.recovery_yaw_std * cfg.recovery_yaw_std * j_yaw * j_yaw.transpose();
        system->inflateCovariance(extra_cov);

        // A fall is also a chance to have been turned around without the landmarks noticing, and
        // they can never notice: the two symmetric field poses fit each other's landmarks
        // identically. Re-seed the mirror so any out-of-field evidence has something to switch to.
        if (cfg.use_hypothesis_bank && system->numHypotheses() == 1) {
            system->spawnMirror();
        }

        log<INFO>("Recovered after ",
                  t - fall_start_t,
                  "s not upright; inflated to sigma_xy ",
                  std::sqrt(system->density.cov()(0, 0)),
                  "m, sigma_yaw ",
                  std::sqrt(filter::SystemLocalisation::yawVariance(system->density.mean(), system->density.cov()))
                      * 180.0 / M_PI,
                  "deg");
    }

    const filter::SensorsSample* FieldLocalisationSRIF::nearest_sensors(double t) const {
        const filter::SensorsSample* best = nullptr;
        double best_dt                    = std::numeric_limits<double>::infinity();
        for (const filter::SensorsSample& s : sensors_window) {
            const double dt = std::abs(s.t - t);
            if (dt < best_dt) {
                best_dt = dt;
                best    = &s;
            }
        }
        return (best != nullptr && best_dt <= cfg.max_sensor_pairing_age) ? best : nullptr;
    }

    bool FieldLocalisationSRIF::solve_initial_pose(const filter::VisionSample& sample,
                                                   const filter::Pose<double>& Tbc,
                                                   const filter::Pose<double>& Twt,
                                                   Eigen::Matrix<double, 9, 1>& eta0) const {
        constexpr Eigen::Index nx = filter::SystemLocalisation::nx;

        // Roll, pitch and torso height come from the gravity-aligned kinematic chain.
        const Eigen::Vector3d rpy_torso = rot2rpy(Twt.rotationMatrix);
        const double roll0              = rpy_torso.x();
        const double pitch0             = rpy_torso.y();
        const double z0                 = Twt.translationVector.z();

        const filter::FieldDimensions& dims = map->dims;
        const double half_length            = dims.fieldLength / 2.0 + dims.borderStripMinWidth;
        const double half_width             = dims.fieldWidth / 2.0 + dims.borderStripMinWidth;

        // Probe system used only to drive association/likelihood scoring (no prediction).
        const std::vector<filter::BodyTwistSample> no_twist;
        const Eigen::MatrixXd S_probe = Eigen::MatrixXd::Identity(nx, nx) * 0.01;
        filter::SystemLocalisation probe(GaussianInfo<double>::fromSqrtMoment(Eigen::VectorXd::Zero(nx), S_probe),
                                         no_twist);

        double best_score        = -std::numeric_limits<double>::infinity();
        std::size_t best_assoc   = 0;
        Eigen::VectorXd best_eta = Eigen::VectorXd::Zero(nx);

        for (double x = -half_length; x <= half_length; x += cfg.grid_step_xy) {
            for (double y = -half_width; y <= half_width; y += cfg.grid_step_xy) {
                for (double yaw = -M_PI; yaw < M_PI; yaw += cfg.grid_step_yaw) {
                    Eigen::VectorXd candidate(nx);
                    candidate << x, y, z0, rpy2quat(Eigen::Vector3d(roll0, pitch0, yaw)), 0.0, 0.0;
                    probe.resetTo(GaussianInfo<double>::fromSqrtMoment(candidate, S_probe), sample.t);

                    filter::MeasurementFieldLandmarks measurement(sample.t, sample, Tbc, *map, probe, cfg.measurement);
                    if (measurement.numAssociated() < std::size_t(cfg.min_init_associations)) {
                        continue;
                    }
                    // The robust likelihood already rewards inliers and floors outliers at the clutter
                    // density, so it favours the pose that explains the most rays well.
                    const double score = measurement.logLikelihood(candidate, probe);
                    if (score > best_score) {
                        best_score = score;
                        best_assoc = measurement.numAssociated();
                        best_eta   = candidate;
                    }
                }
            }
        }

        if (best_assoc < std::size_t(cfg.min_init_associations)) {
            return false;
        }

        // Resolve the own-half/opponent-half symmetry from game context. The mirror has an identical
        // likelihood, so this loses no fit; it only picks the physically valid side.
        if (cfg.own_half_x_sign != 0.0 && best_eta(0) * cfg.own_half_x_sign < 0.0) {
            best_eta = filter::SystemLocalisation::mirrorState(best_eta);
        }

        eta0 = best_eta;
        return true;
    }

    void FieldLocalisationSRIF::emit_field(const filter::Pose<double>& Htw,
                                           const filter::MeasurementFieldLandmarks* measurement) {
        const Eigen::VectorXd mean = system->density.mean();

        // World-to-field transform: Tfb(mean) * Htw (torso-in-field composed with world-to-torso).
        const filter::Pose<double> Tfb   = filter::SystemLocalisation::fieldPose<double>(mean);
        const Eigen::Isometry3d Hfw_full = to_isometry(Tfb * Htw);
        auto field                       = std::make_unique<Field>();

        // Emit the planar (ground-plane) world-to-field pose: keep (x, y) and the yaw, drop the torso
        // height and roll/pitch. The field is a flat z=0 model, so a full SE(3) Hfw (which carries the
        // walking torso's tilt and ~0.4 m height) tips the field lines off the plane in NUsight. Matching
        // the NLopt convention (Hfw = Translation(x, y, 0) * Rz(yaw)) keeps them flat, and every consumer
        // reasons about the field on the ground plane anyway.
        const double yaw      = rot2rpy(Hfw_full.rotation()).z();
        Eigen::Isometry3d Hfw = Eigen::Isometry3d::Identity();
        Hfw.linear()          = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Hfw.translation()     = Eigen::Vector3d(Hfw_full.translation().x(), Hfw_full.translation().y(), 0.0);
        field->Hfw            = Hfw;

        // TODO(future): localise the feet in the field frame as well. This emits only the torso field
        // pose (Hfw), which is enough to place the robot, but ball passing and foot placement would
        // benefit from knowing where each foot is on the field. The feet field poses could be derived by
        // composing the kinematic foot frames (Sensors.Htx[L_FOOT_BASE]/[R_FOOT_BASE]) with the field
        // pose, then emitted alongside (or added to) the Field message. Deferred - future work.

        // Covariance / uncertainty of the reported (x, y, yaw). Position is still the leading 2x2
        // block, but yaw is no longer a state element: it is a direction in the quaternion block,
        // so both its variance and its cross-covariance with position go through row 2 of the
        // attitude Jacobian rather than through index 5.
        const Eigen::MatrixXd P           = system->density.cov();
        constexpr Eigen::Index iq         = filter::SystemLocalisation::iQuat;
        const Eigen::RowVector4d g_yaw    = filter::SystemLocalisation::attitudeJacobian(mean).row(2);
        const Eigen::Vector2d cov_pos_yaw = P.block<2, 4>(0, iq) * g_yaw.transpose();
        const double var_yaw              = g_yaw * P.block<4, 4>(iq, iq) * g_yaw.transpose();
        Eigen::Matrix3d covariance;
        // clang-format off
        covariance << P(0, 0),          P(0, 1),          cov_pos_yaw(0),
                      P(1, 0),          P(1, 1),          cov_pos_yaw(1),
                      cov_pos_yaw(0),   cov_pos_yaw(1),   var_yaw;
        // clang-format on
        field->covariance  = covariance;
        field->uncertainty = covariance.trace();

        // Hypotheses as (x, y, yaw) particles (a single component in single-hypothesis mode).
        if (system->numHypotheses() > 1) {
            for (const GaussianInfo<double>& component : system->hypotheses()) {
                const Eigen::VectorXd m = component.mean();
                field->particles.emplace_back(m(0), m(1), filter::SystemLocalisation::heading(m));
            }
        }
        else {
            field->particles.emplace_back(mean(0), mean(1), filter::SystemLocalisation::heading(mean));
        }

        // Cost: mean chordal angular residual of the associated rays at the posterior mean [rad].
        double cost = 0.0;
        if (measurement != nullptr && measurement->numAssociated() > 0) {
            const Eigen::Matrix<double, 3, Eigen::Dynamic>& measured = measurement->measuredRays();
            const Eigen::Matrix<double, 3, Eigen::Dynamic> predicted = measurement->predictRays<double>(mean);
            double residual                                          = 0.0;
            for (Eigen::Index j = 0; j < measured.cols(); ++j) {
                residual += std::acos(std::clamp(measured.col(j).dot(predicted.col(j)), -1.0, 1.0));
            }
            cost = residual / double(measured.cols());
        }
        field->cost = cost;

        emit(field);

        if (log_level <= DEBUG) {
            emit(graph("SRIF field pose (x, y, yaw)", mean(0), mean(1), filter::SystemLocalisation::heading(mean)));
            emit(graph("SRIF uncertainty", field->uncertainty));
            emit(graph("SRIF cost (mean ray residual)", cost));
            if (measurement != nullptr) {
                emit(graph("SRIF associations", double(measurement->numAssociated())));
            }
        }
    }

}  // namespace module::localisation
