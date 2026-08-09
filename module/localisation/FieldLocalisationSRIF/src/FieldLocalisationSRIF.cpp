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
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "measurement/MeasurementBodyRates.hpp"
#include "measurement/MeasurementGravity.hpp"
#include "measurement/MeasurementKinematicHeight.hpp"
#include "measurement/MeasurementQuaternionNorm.hpp"
#include "srif/FieldMapFromDescription.hpp"

#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/input/Image.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/BoundingBoxes.hpp"
#include "message/vision/OutOfFieldFeatures.hpp"

#include "utility/gaussian_filtering/gaussian/GaussianInfo.hpp"
#include "utility/gaussian_filtering/rotation.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/Vision.hpp"
#include "utility/vision/fourcc.hpp"
#include "utility/vision/projection.hpp"

namespace module::localisation {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::input::Image;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::localisation::ResetFieldLocalisation;
    using message::support::FieldDescription;
    using message::vision::BoundingBox;
    using message::vision::BoundingBoxes;
    using message::vision::OutOfFieldFeature;
    using message::vision::OutOfFieldFeatures;
    using message::vision::OutOfFieldLandmark;

    using measurement::MeasurementBodyVelocity;
    using measurement::MeasurementGravity;
    using measurement::MeasurementGyroscope;
    using measurement::MeasurementKinematicHeight;
    using measurement::MeasurementQuaternionNorm;
    using utility::gaussian_filtering::rot2rpy;
    using utility::gaussian_filtering::rpy2quat;
    using utility::gaussian_filtering::gaussian::GaussianInfo;
    using utility::nusight::graph;
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

    /// @brief Single-channel 8-bit view of a camera frame, for FAST/ORB.
    ///
    /// Handles the formats the two cameras this runs on actually deliver: BGR3 and Bayer RGGB from
    /// the hardware Camera module, RGBA from Webots. The corner detector only ever wanted intensity,
    /// so a demosaic to colour and back would be wasted work -- Bayer is converted straight to grey.
    ///
    /// @return False if the format is not one we can read, leaving `gray` untouched.
    static bool to_grayscale(const Image& image, cv::Mat& gray) {
        const int width  = int(image.dimensions.x());
        const int height = int(image.dimensions.y());
        // The Mat is a view over the message's buffer; cvtColor allocates the output, so nothing
        // here outlives the message.
        uint8_t* data = const_cast<uint8_t*>(image.data.data());
        switch (image.format) {
            case utility::vision::fourcc("BGR3"):
                cv::cvtColor(cv::Mat(height, width, CV_8UC3, data), gray, cv::COLOR_BGR2GRAY);
                return true;
            case utility::vision::FOURCC::RGGB:
                cv::cvtColor(cv::Mat(height, width, CV_8UC1, data), gray, cv::COLOR_BayerRG2GRAY);
                return true;
            case utility::vision::fourcc("RGBA"):
                cv::cvtColor(cv::Mat(height, width, CV_8UC4, data), gray, cv::COLOR_RGBA2GRAY);
                return true;
            case utility::vision::FOURCC::GREY: cv::Mat(height, width, CV_8UC1, data).copyTo(gray); return true;
            default: return false;
        }
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

            // Only the knobs worth reaching for live in the yaml; everything else is a tuned constant
            // in Config, next to the reasoning for its value. See the comment on Config.

            // What the filter runs
            cfg.use_hypothesis_bank    = config["use_hypothesis_bank"].as<bool>();
            cfg.use_side_disambiguator = config["use_side_disambiguator"].as<bool>();
            cfg.use_odometry_velocity  = config["use_odometry_velocity"].as<bool>();
            cfg.use_gravity            = config["use_gravity"].as<bool>();
            cfg.use_kinematic_height   = config["use_kinematic_height"].as<bool>();

            // How far each sensor is trusted
            cfg.gyroscope_sigma         = config["gyroscope_sigma"].as<double>();
            cfg.odometry_velocity_sigma = config["odometry_velocity_sigma"].as<double>();
            cfg.gravity_sigma           = config["gravity_sigma"].as<double>();
            cfg.height_sigma            = config["height_sigma"].as<double>();

            // How far vision is trusted
            cfg.measurement.sigmaAngular  = config["measurement"]["sigma_angular"].as<double>();
            cfg.measurement.gateAngle     = config["measurement"]["gate_angle"].as<double>();
            cfg.measurement.minConfidence = config["measurement"]["min_confidence"].as<double>();

            // How fast the belief may move. These two dominate: uncertainty reaches position by
            // integrating velocity uncertainty, so the pose PSDs are only a floor against collapse.
            cfg.process.sigmaVel   = config["process"]["sigma_vel"].as<double>();
            cfg.process.sigmaOmega = config["process"]["sigma_omega"].as<double>();

            // How much confidence a fall costs
            cfg.recovery_pos_std = config["fall"]["recovery_pos_std"].as<double>();
            cfg.recovery_yaw_std = config["fall"]["recovery_yaw_std"].as<double>();

            if ((cfg.initial_sqrt_covariance.array() < 1e-4).any()) {
                log<WARN>("initial_sqrt_covariance has a near-zero entry; the filter stores its inverse");
            }

            // Propagate live tuning into an already-constructed estimator
            if (system != nullptr) {
                system->params = cfg.process;
                system->hyp    = cfg.hypothesis;
            }
        });

        on<Startup, Trigger<FieldDescription>>().then("Build field landmark map", [this](const FieldDescription& fd) {
            map = std::make_unique<filter::FieldMap>(srif::field_dimensions(fd));
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
            // Raw, and kept raw: the gyroscope is its own measurement of omegaBb now
            // (MeasurementGyroscope), not a substitute for the odometry's angular rate.
            s.gyroscope = sensors.gyroscope;
            sensors_window.push_back(std::move(s));

            // Drop samples older than the configured window (keep at least two to difference across)
            const double cutoff = sensors_window.back().t - cfg.twist_window_seconds;
            auto first_kept     = std::find_if(sensors_window.begin(), sensors_window.end(), [cutoff](const auto& e) {
                return e.t >= cutoff;
            });
            if (first_kept != sensors_window.begin() && std::distance(first_kept, sensors_window.end()) >= 2) {
                sensors_window.erase(sensors_window.begin(), first_kept);
            }

            // Body-fixed velocity samples for MeasurementBodyVelocity. Nothing is a known input
            // to the process model, so this is simply a buffer the vision reaction reads the
            // nearest sample from.
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

                // Posture drives a per-model gate. Each measurement is
                // suppressed only where its own assumption breaks: kinematic height needs the
                // support leg on the ground, gravity needs the torso not to be accelerating.
                // Landmarks and the gyroscope are valid face-down, and a fall is when the
                // estimate is most at risk -- a robot that spins while toppling or getting up
                // changes its heading, and only measurements taken during the event catch it.
                //
                // Absent a Stability message the robot is assumed upright, so fall handling
                // simply never engages.
                const bool upright = stability == nullptr || *stability > Stability::FALLING;

                // Transitions are handled before any early return below, and must stay that way:
                // a getup ends in motion blur, so the frame the robot first reads upright again
                // often carries no usable detections. Handling it further down would skip the
                // recovery inflation on exactly those falls. Doing it here also means setPosture
                // sees this episode's fall_start_t rather than the previous one's.
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

                // Body rates go in before the no-detections gate below: neither depends on YOLO
                // finding anything, and a fallen robot's camera is in the carpet, so those are
                // exactly the frames where an unmeasured velocity state would integrate the
                // pre-fall gait straight off the field. Measurement::process predicts to t
                // itself, so the predictAll further down is a zero-dt no-op once these have run.
                if (initialised) {
                    // Valid whatever the posture: the gyroscope measures a topple honestly.
                    if (paired->gyroscope.allFinite()) {
                        MeasurementGyroscope gyro(t, paired->gyroscope, cfg.gyroscope_sigma);
                        system->process(gyro);
                    }

                    // The walk-engine odometry describes the gait the engine believes it is
                    // executing. Upright that is loose but real information; on the ground it is
                    // fiction, and during a getup a scripted flail that is not locomotion. So
                    // while not upright it is replaced by a zero-velocity update.
                    if (upright) {
                        const filter::BodyTwistSample* twist = nearest_twist(t);
                        if (cfg.use_odometry_velocity && twist != nullptr && twist->vBb.allFinite()) {
                            MeasurementBodyVelocity vel(t, twist->vBb, cfg.odometry_velocity_sigma);
                            system->process(vel);
                        }
                    }
                    else {
                        // A robot on the carpet is not travelling anywhere. Saying so is what
                        // stops the pre-fall walking velocity integrating across the whole fall.
                        // Leaving vBb unmeasured instead is not the neutral choice it looks like:
                        // it asserts the robot may still be moving at whatever it was doing when
                        // it fell. Lying still it really is stationary (zupt_sigma); mid-topple
                        // and mid-getup the torso moves, just not anywhere (zupt_dynamic_sigma).
                        const bool settled = stability != nullptr && *stability == Stability::FALLEN;
                        MeasurementBodyVelocity zupt =
                            MeasurementBodyVelocity::stationary(t, settled ? cfg.zupt_sigma : cfg.zupt_dynamic_sigma);
                        system->process(zupt);
                    }
                }

                // A face-down fall produces no detections at all. Prediction otherwise happens
                // only inside Event::process, so such a frame would advance neither the state nor
                // the clock, and the filter would leave the fall holding its pre-fall mean at its
                // pre-fall covariance -- confidently wrong rather than honestly uncertain.
                if (sample.detections.empty()) {
                    if (initialised) {
                        system->predictAll(t);
                    }
                    return;
                }

                // Bootstrap: solve a coarse initial pose the first time we see usable landmarks.
                if (!initialised) {
                    const filter::Pose<double> Twt = Htw.inverse();
                    Eigen::Matrix<double, 18, 1> eta0;
                    if (!solve_initial_pose(sample, Tbc, Twt, eta0)) {
                        log<DEBUG>("Initial pose solve did not associate enough landmarks yet");
                        return;
                    }

                    Eigen::MatrixXd S0 =
                        Eigen::MatrixXd::Zero(filter::SystemLocalisation::nx, filter::SystemLocalisation::nx);
                    S0.diagonal() = cfg.initial_sqrt_covariance;
                    const auto p0 = GaussianInfo<double>::fromSqrtMoment(Eigen::VectorXd(eta0), S0);

                    system         = std::make_unique<filter::SystemLocalisation>(p0);
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

                // Advance to the vision capture time. The process model is autonomous, driven by
                // the velocity states rather than by any input.
                system->predict(t);

                // Unit-norm pseudo-measurement. The four attitude states carry three degrees of
                // freedom and quat2rot normalises, so |q| is invisible to every other model here;
                // without this the MAP Hessian is singular along it. Applied first so the rest of
                // the frame's updates see a belief that is on the sphere.
                MeasurementQuaternionNorm quaternion_norm(t, cfg.quaternion_norm_sigma);
                system->process(quaternion_norm);

                // Landmark measurement update (routed through process() so the hypothesis bank,
                // when active, applies it to every mixture component and reweights them).
                filter::MeasurementFieldLandmarks measurement(t, sample, Tbc, *map, *system, cfg.measurement);
                system->process(measurement);

                // Gravity is valid whenever the torso is not being accelerated -- true of a robot
                // lying still on the carpet, false of one in free fall or hitting the ground,
                // whatever its posture. Gating on the specific-force magnitude tests that
                // condition directly, which beats gating on "upright" both during a fall and
                // while walking.
                if (cfg.use_gravity && paired->accelerometer.allFinite()) {
                    constexpr double standard_gravity = 9.80665;
                    const double a_mag                = paired->accelerometer.norm();
                    if (std::abs(a_mag - standard_gravity) < cfg.gravity_quasi_static_tolerance) {
                        MeasurementGravity gravity(t, paired->accelerometer, cfg.gravity_sigma);
                        system->process(gravity);
                    }
                }
                // Torso height assumes the support leg reaches the ground, so this is the one
                // model a fall genuinely invalidates: lying down, the chain still reports a
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

        // Out-of-field side disambiguation runs off the raw camera frame rather than the YOLO
        // boxes: it needs the pixels for FAST/ORB, and the image carries its own lens and
        // capture-time Hcw, so it can pair itself against the odometry exactly as the landmark
        // path does. Kept as its own reaction (and its own Single) so a slow frame drops corner
        // detection instead of delaying a landmark update.
        on<Trigger<Image>, Sync<FieldLocalisationSRIF>, Single>().then(
            "SRIF out-of-field side disambiguation",
            [this](const Image& image) { run_side_disambiguation(image); });
    }

    void FieldLocalisationSRIF::run_side_disambiguation(const Image& image) {
        // Nothing to disambiguate before there is a pose and a field to compare against.
        if (!cfg.use_side_disambiguator || map == nullptr || !initialised || !have_t0) {
            return;
        }

        cv::Mat gray;
        if (!to_grayscale(image, gray)) {
            log<WARN>("Out-of-field disambiguation: unsupported image format ", utility::vision::fourcc(image.format));
            return;
        }

        // Built on the first frame, not at configuration time: the lens calibration and the image
        // size travel with the image, so this is the first point at which they are known and
        // correct for whichever camera is actually running.
        if (side == nullptr) {
            side = std::make_unique<filter::SideDisambiguator>(
                image.lens,
                Eigen::Vector2d(double(image.dimensions.x()), double(image.dimensions.y())),
                map->dims);
        }

        const double t = seconds(image.timestamp);

        // Same pairing as the landmark path: Hcw is at capture, so the torso pose must be too.
        const filter::SensorsSample* paired = nearest_sensors(t);
        if (paired == nullptr) {
            log<DEBUG>("No odometry sample near the camera frame; skipping side disambiguation");
            return;
        }
        const filter::Pose<double> Tbc = paired->Htw * to_pose(Eigen::Isometry3d(image.Hcw)).inverse();

        // Camera pose in {f} at the posterior mean, and under the mirrored state. Same kinematics
        // both times -- only the torso pose is mirrored, which is exactly the ambiguity being tested.
        const Eigen::VectorXd mean = system->density.mean();
        auto camera_pose           = [&Tbc](const Eigen::VectorXd& x) {
            return filter::SystemLocalisation::fieldPose<double>(x) * Tbc
                   * filter::Pose<double>(filter::SystemLocalisation::cameraBiasRotation<double>(x),
                                          Eigen::Vector3d::Zero());
        };
        const Eigen::VectorXd mirror = filter::SystemLocalisation::mirrorState(mean);

        // Gating inputs: how uncertain the filter thinks it is, and how fast it is turning.
        const Eigen::MatrixXd P   = system->density.cov();
        const double pos_std      = std::sqrt(std::max(P(0, 0), P(1, 1)));
        const double yaw_std      = std::sqrt(filter::SystemLocalisation::yawVariance(mean, P));
        const double yaw_rate_abs = std::abs(paired->gyroscope.z());

        const filter::SideDisambiguator::FrameResult result = side->process(t,
                                                                            gray,
                                                                            camera_pose(mean),
                                                                            camera_pose(mirror),
                                                                            pos_std,
                                                                            yaw_std,
                                                                            yaw_rate_abs,
                                                                            filter::SystemLocalisation::heading(mean));

        if (cfg.use_hypothesis_bank) {
            // Bank mode: the background evidence is the only thing that separates the two symmetric
            // hypotheses (landmarks leave them at 50/50), so fold each frame's log-ratio into the
            // mixture weights. The representative switching sides IS the correction, done smoothly
            // by the weights rather than by a discontinuous state flip.
            system->addSideLogEvidence(result.sideDelta);

            // If the mirror has already been pruned and the evidence now says the surviving belief
            // is wrong (a mid-game kidnap), re-seed the alternative so the weights have something to
            // switch to. The cooldown stops repeated spawns while flipRequested stays latched -- the
            // LLR takes a few seconds to climb back after a switch.
            if (result.flipRequested && system->numHypotheses() == 1
                && t - last_respawn_t > side->options.flipCooldown) {
                system->spawnMirror();
                last_respawn_t = t;
                log<INFO>("Out-of-field evidence (llr ", result.llr, ") re-seeded the mirror hypothesis");
            }
        }
        else if (result.flipRequested) {
            // Single-hypothesis mode: the background says we are on the wrong side, so mirror the
            // belief outright and tell the disambiguator, which negates its accumulated evidence and
            // freezes map building while the estimator re-converges.
            system->resetTo(filter::SystemLocalisation::mirrorDensity(system->density), t);
            side->notifyFlipApplied(t);
            const Eigen::VectorXd flipped = system->density.mean();
            log<INFO>("Out-of-field side flip (llr ",
                      result.llr,
                      ", assoc ",
                      result.nAssociated,
                      "/",
                      result.nAssociatedMirror,
                      " own/mirror): corrected to x=",
                      flipped(0),
                      "m y=",
                      flipped(1),
                      "m yaw=",
                      filter::SystemLocalisation::heading(flipped) * 180.0 / M_PI,
                      "deg");
        }

        emit_out_of_field(image, result);

        if (log_level <= DEBUG) {
            emit(graph("SRIF/side llr", result.llr));
            emit(graph("SRIF/side associations own-mirror",
                       double(result.nAssociated),
                       double(result.nAssociatedMirror)));
            emit(graph("SRIF/side landmarks", double(result.nLandmarks), double(result.nCandidates)));
        }
    }

    void FieldLocalisationSRIF::emit_out_of_field(const Image& image,
                                                  const filter::SideDisambiguator::FrameResult& result) {
        auto msg       = std::make_unique<OutOfFieldFeatures>();
        msg->id        = image.id;
        msg->timestamp = image.timestamp;
        msg->Hcw       = image.Hcw;

        // The status enums are declared in the same precedence order on both sides, so the mapping
        // is positional. Static-assert the ends so a value inserted in either list is a build error
        // rather than a silently miscoloured overlay.
        using FeatureStatus  = OutOfFieldFeature::Status;
        using LandmarkStatus = OutOfFieldLandmark::Status;
        static_assert(int(FeatureStatus::ON_CARPET) == filter::SideDisambiguator::FEATURE_ON_CARPET
                          && int(FeatureStatus::ASSOCIATED) == filter::SideDisambiguator::FEATURE_ASSOCIATED,
                      "OutOfFieldFeature::Status has diverged from SideDisambiguator::FeatureStatus");
        static_assert(int(LandmarkStatus::NOT_IN_VIEW) == filter::SideDisambiguator::LANDMARK_NOT_IN_VIEW
                          && int(LandmarkStatus::CULLED_OUTLIER) == filter::SideDisambiguator::LANDMARK_CULLED_OUTLIER,
                      "OutOfFieldLandmark::Status has diverged from SideDisambiguator::LandmarkStatus");

        msg->features.reserve(result.features.size());
        for (std::size_t i = 0; i < result.features.size(); ++i) {
            OutOfFieldFeature feature;
            feature.uPCc   = result.features[i].uPCc;
            feature.status = FeatureStatus(FeatureStatus::Value(result.featureStatus[i]));
            msg->features.push_back(feature);
        }

        // Landmarks are sent as rays rather than pixels so NUsight can re-project them through the
        // lens it already holds, and so a landmark that has drifted outside the image still draws.
        const Eigen::Vector2d dimensions(double(image.dimensions.x()), double(image.dimensions.y()));
        msg->landmarks.reserve(result.landmarkViews.size());
        for (const filter::SideDisambiguator::LandmarkView& view : result.landmarkViews) {
            OutOfFieldLandmark landmark;
            landmark.uPCc         = utility::vision::unproject_pixel(view.px, image.lens, dimensions);
            landmark.status       = LandmarkStatus(LandmarkStatus::Value(view.status));
            landmark.bearing_only = view.far;
            if (view.status == filter::SideDisambiguator::LANDMARK_ASSOCIATED) {
                landmark.uMatchCc = utility::vision::unproject_pixel(view.matchPx, image.lens, dimensions);
            }
            msg->landmarks.push_back(landmark);
        }

        msg->llr             = result.llr;
        msg->side_delta      = result.sideDelta;
        msg->landmark_count  = uint32_t(result.nLandmarks);
        msg->candidate_count = uint32_t(result.nCandidates);
        msg->map_frozen      = result.mapFrozen;
        msg->flip_requested  = result.flipRequested;

        emit(msg);
    }

    const filter::BodyTwistSample* FieldLocalisationSRIF::nearest_twist(double t) const {
        const filter::BodyTwistSample* best = nullptr;
        double best_dt                      = std::numeric_limits<double>::infinity();
        for (const filter::BodyTwistSample& s : twist_buffer) {
            const double dt = std::abs(s.t - t);
            if (dt < best_dt) {
                best_dt = dt;
                best    = &s;
            }
        }
        return best_dt <= cfg.max_sensor_pairing_age ? best : nullptr;
    }

    void FieldLocalisationSRIF::apply_fall_recovery(double t) {
        // The mean is kept: a fall and getup move the torso well under a metre, so the pre-fall
        // position is still the best estimate available. Re-solving globally would be worse, since
        // the grid search breaks the field symmetry using the known starting half -- a prior that
        // is false once play is under way. What a fall destroys is confidence, above all in yaw,
        // so that is what is handed back. The widened belief is also what reopens the landmark
        // association gate (MeasurementFieldLandmarks::Options::gateYawScale); without it a getup
        // that turned the robot leaves every predicted bearing outside the gate and the filter
        // can never re-acquire.
        const Eigen::VectorXd xr = system->density.mean();
        Eigen::MatrixXd extra_cov =
            Eigen::MatrixXd::Zero(filter::SystemLocalisation::nx, filter::SystemLocalisation::nx);
        extra_cov(0, 0) = extra_cov(1, 1) = cfg.recovery_pos_std * cfg.recovery_pos_std;
        // Yaw uncertainty is about the field z axis, which lands on the quaternion states as a
        // rank-one block rather than a single diagonal element: no element is "the yaw".
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
                                                   Eigen::Matrix<double, 18, 1>& eta0) const {
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
        const Eigen::MatrixXd S_probe = Eigen::MatrixXd::Identity(nx, nx) * 0.01;
        filter::SystemLocalisation probe(GaussianInfo<double>::fromSqrtMoment(Eigen::VectorXd::Zero(nx), S_probe));

        double best_score        = -std::numeric_limits<double>::infinity();
        std::size_t best_assoc   = 0;
        Eigen::VectorXd best_eta = Eigen::VectorXd::Zero(nx);

        for (double x = -half_length; x <= half_length; x += cfg.grid_step_xy) {
            for (double y = -half_width; y <= half_width; y += cfg.grid_step_xy) {
                for (double yaw = -M_PI; yaw < M_PI; yaw += cfg.grid_step_yaw) {
                    Eigen::VectorXd candidate(nx);
                    // Rates start at zero: the grid solve runs on the first usable frame, and
                    // the body-rate measurements sharpen them within a frame or two.
                    candidate << x, y, z0, rpy2quat(Eigen::Vector3d(roll0, pitch0, yaw)),
                        Eigen::Matrix<double, 9, 1>::Zero(), 0.0, 0.0;
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

        // Resolve the own-half/opponent-half symmetry. The mirror has an identical likelihood, so
        // this loses no fit; it only picks the physically valid side.
        //
        // Own half is +x by the field-frame convention the rest of the codebase already hardcodes:
        // our goal sits at +field_length/2 (Defend, Goalie, ReadyAttack, FieldLocalisationNLopt's
        // own_goal_posts) and the goal we attack at -field_length/2 (WalkToBall, PenaltyShootout).
        // The frame is defined relative to our own goal and nothing swaps it by team or half, so
        // this is a fixed fact about the frame rather than a per-game setting -- and the rules put
        // every robot in its own half at kickoff, which is what makes the prior true at init and
        // false afterwards.
        if (best_eta(0) < 0.0) {
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

        // Covariance of the reported (x, y, yaw). Position is the leading 2x2 block; yaw is not a
        // state element but a direction in the quaternion block, so both its variance and its
        // cross-covariance with position go through row 2 of the attitude Jacobian.
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
