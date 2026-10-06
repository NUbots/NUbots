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
    /// @return False if the format is not one we can read, leaving `gray` untouched.
    static bool to_grayscale(const Image& image, cv::Mat& gray) {
        const int width  = int(image.dimensions.x());
        const int height = int(image.dimensions.y());
        // The Mat is a view over the message's buffer; cvtColor allocates the output.
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
    /// Corner rays are ordered TL, TR, BR, BL, as MeasurementFieldLandmarks::detectionRay expects.
    /// Class filtering and the confidence threshold are applied there, not here.
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

            // What the filter runs
            cfg.use_hypothesis_bank    = config["use_hypothesis_bank"].as<bool>();
            cfg.use_side_disambiguator = config["use_side_disambiguator"].as<bool>();
            cfg.use_gravity            = config["use_gravity"].as<bool>();
            cfg.use_kinematic_height   = config["use_kinematic_height"].as<bool>();

            // How far each sensor is trusted
            cfg.gyroscope_sigma         = config["gyroscope_sigma"].as<double>();
            cfg.odometry_velocity_sigma = config["odometry_velocity_sigma"].as<double>();
            const std::string source    = config["odometry_velocity_source"].as<std::string>();
            if (source == "SENSORS_VTW") {
                cfg.odometry_velocity_source = Config::OdometryVelocitySource::SENSORS_VTW;
            }
            else {
                if (source != "HTW_DIFFERENCE") {
                    log<ERROR>("Unknown odometry_velocity_source '", source, "'; using HTW_DIFFERENCE");
                }
                cfg.odometry_velocity_source = Config::OdometryVelocitySource::HTW_DIFFERENCE;
            }
            cfg.gravity_sigma = config["gravity_sigma"].as<double>();
            cfg.height_sigma  = config["height_sigma"].as<double>();

            // How far vision is trusted
            cfg.measurement.sigmaAngular  = config["measurement"]["sigma_angular"].as<double>();
            cfg.measurement.gateAngle     = config["measurement"]["gate_angle"].as<double>();
            cfg.measurement.minConfidence = config["measurement"]["min_confidence"].as<double>();

            // How fast the belief may move
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

        // Maintain the rolling odometry window. Priority::HIGH and the shared Sync group keep
        // sensors_window from being mutated while the vision reaction reads it.
        on<Trigger<Sensors>, Sync<FieldLocalisationSRIF>, Priority::HIGH>().then([this](const Sensors& sensors) {
            if (!have_t0) {
                t0      = sensors.timestamp;
                have_t0 = true;
            }

            filter::SensorsSample s;
            s.t             = seconds(sensors.timestamp);
            s.Htw           = to_pose(Eigen::Isometry3d(sensors.Htw));
            s.accelerometer = sensors.accelerometer;
            // Kept raw: the gyroscope is its own measurement (MeasurementGyroscope).
            s.gyroscope = sensors.gyroscope;

            // The velocity this sample hands MeasurementBodyVelocity. Either branch may fail to
            // produce one, and says so with a non-finite vBb.
            s.vBb = cfg.odometry_velocity_source == Config::OdometryVelocitySource::SENSORS_VTW
                        // vTw is world-frame; Htw's rotation takes world to torso.
                        ? Eigen::Vector3d(s.Htw.rotationMatrix * Eigen::Vector3d(sensors.vTw))
                        : sensors_window.empty()
                              ? Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())
                              : filter::SystemLocalisation::bodyVelocityFromOdometry(sensors_window.back(),
                                                                                     s,
                                                                                     cfg.max_odometry_gap);

            sensors_window.push_back(std::move(s));

            // Drop samples older than the window, keeping at least two
            const double cutoff = sensors_window.back().t - cfg.sensors_window_seconds;
            auto first_kept     = std::find_if(sensors_window.begin(), sensors_window.end(), [cutoff](const auto& e) {
                return e.t >= cutoff;
            });
            if (first_kept != sensors_window.begin() && std::distance(first_kept, sensors_window.end()) >= 2) {
                sensors_window.erase(sensors_window.begin(), first_kept);
            }
        });

        on<Trigger<BoundingBoxes>, Optional<With<Stability>>, Sync<FieldLocalisationSRIF>, Single>().then(
            "SRIF field localisation",
            [this](const BoundingBoxes& boxes, const std::shared_ptr<const Stability>& stability) {
                // Prerequisites: map built and odometry seen.
                if (map == nullptr || !have_t0) {
                    return;
                }

                const double t = seconds(boxes.timestamp);

                // Posture gates each measurement separately, where its own assumption breaks.
                // Absent a Stability message the robot is assumed upright.
                const bool upright = stability == nullptr || *stability > Stability::FALLING;

                // Must stay ahead of every early return below: the first upright frame after a
                // getup often carries no usable detections.
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

                // Pair the odometry to the vision capture time: Hcw is at capture, so Htw must be
                // too, or the capture-to-now torso motion lands in Tbc.
                const filter::SensorsSample* paired = nearest_sensors(t);
                if (paired == nullptr) {
                    log<DEBUG>("No odometry sample near the vision frame; skipping");
                    if (initialised) {
                        system->predictAll(t);
                    }
                    return;
                }
                const filter::Pose<double>& Htw = paired->Htw;

                // Camera pose w.r.t. torso: Tbc = Htw * Hcw^{-1} (the odometry world cancels).
                const filter::Pose<double> Tbc = Htw * to_pose(Eigen::Isometry3d(boxes.Hcw)).inverse();

                const filter::VisionSample sample = build_vision_sample(t, boxes);

                // Ahead of the no-detections gate below: neither rate depends on YOLO finding
                // anything. Measurement::process predicts to t, so the predictAll below is then a
                // zero-dt no-op.
                if (initialised) {
                    // Valid whatever the posture.
                    if (paired->gyroscope.allFinite()) {
                        MeasurementGyroscope gyro(t, paired->gyroscope, cfg.gyroscope_sigma);
                        system->process(gyro);
                    }

                    // The odometry velocity is only meaningful while upright; off the feet it is
                    // replaced by a zero-velocity update.
                    if (upright) {
                        if (paired->vBb.allFinite()) {
                            MeasurementBodyVelocity vel(t, paired->vBb, cfg.odometry_velocity_sigma);
                            system->process(vel);
                        }
                    }
                    else {
                        // Lying still the robot really is stationary (zupt_sigma); mid-topple and
                        // mid-getup the torso moves, just not anywhere (zupt_dynamic_sigma).
                        const bool settled = stability != nullptr && *stability == Stability::FALLEN;
                        MeasurementBodyVelocity zupt =
                            MeasurementBodyVelocity::stationary(t, settled ? cfg.zupt_sigma : cfg.zupt_dynamic_sigma);
                        system->process(zupt);
                    }
                }

                // Prediction otherwise happens only inside Event::process, so a frame with no
                // detections would advance neither the state nor the clock.
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

                // Advance to the vision capture time.
                system->predict(t);

                // Unit-norm pseudo-measurement, without which the MAP Hessian is singular along
                // |q|. Applied first so the frame's other updates see a belief on the sphere.
                MeasurementQuaternionNorm quaternion_norm(t, cfg.quaternion_norm_sigma);
                system->process(quaternion_norm);

                // Landmark update. Routed through process() so the hypothesis bank, when active,
                // applies it to every component and reweights them.
                filter::MeasurementFieldLandmarks measurement(t, sample, Tbc, *map, *system, cfg.measurement);
                system->process(measurement);

                // Gravity is valid whenever the torso is not being accelerated, which the
                // specific-force magnitude tests directly. Posture is not the condition.
                if (cfg.use_gravity && paired->accelerometer.allFinite()) {
                    constexpr double standard_gravity = 9.80665;
                    const double a_mag                = paired->accelerometer.norm();
                    if (std::abs(a_mag - standard_gravity) < cfg.gravity_quasi_static_tolerance) {
                        MeasurementGravity gravity(t, paired->accelerometer, cfg.gravity_sigma);
                        system->process(gravity);
                    }
                }
                // Torso height assumes the support leg reaches the ground, so a fall invalidates
                // it outright.
                if (cfg.use_kinematic_height && upright) {
                    const double height = Htw.inverse().translationVector.z();
                    if (std::isfinite(height)) {
                        MeasurementKinematicHeight kinematic_height(t, height, cfg.height_sigma);
                        system->process(kinematic_height);
                    }
                }

                // A non-finite mean would ship a NaN Hfw; re-initialise instead.
                if (!system->density.mean().allFinite()) {
                    log<WARN>("Field localisation state became non-finite; re-initialising");
                    system.reset();
                    initialised = false;
                    return;
                }

                emit_field(Htw, &measurement);
            });

        // Runs off the raw camera frame (FAST/ORB needs the pixels). Its own reaction and its own
        // Single, so a slow frame drops corner detection rather than delaying a landmark update.
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

        // Built on the first frame, not at configuration time: the lens and image size travel
        // with the image.
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

        // Camera pose in {f} at the posterior mean and under the mirrored state. Same kinematics
        // both times; only the torso pose is mirrored.
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
            // Bank mode: fold each frame's log-ratio into the mixture weights. The representative
            // switching sides is the correction, so there is no state flip.
            system->addSideLogEvidence(result.sideDelta);

            // If the mirror was pruned and the evidence now says the survivor is wrong, re-seed
            // the alternative. The cooldown stops repeated spawns while flipRequested is latched.
            if (result.flipRequested && system->numHypotheses() == 1
                && t - last_respawn_t > side->options.flipCooldown) {
                system->spawnMirror();
                last_respawn_t = t;
                log<INFO>("Out-of-field evidence (llr ", result.llr, ") re-seeded the mirror hypothesis");
            }
        }
        else if (result.flipRequested) {
            // Single-hypothesis mode: mirror the belief outright. notifyFlipApplied negates the
            // accumulated evidence and freezes map building while the estimator re-converges.
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

        // Positional mapping between the two status enums. The static_asserts turn a value
        // inserted into either list into a build error rather than a miscoloured overlay.
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

        // Sent as rays rather than pixels so NUsight can re-project them, and so a landmark
        // outside the image still draws.
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

    void FieldLocalisationSRIF::apply_fall_recovery(double t) {
        // The mean is kept and only the confidence is handed back, above all in yaw. The widened
        // belief is also what reopens the landmark association gate
        // (MeasurementFieldLandmarks::Options::gateYawScale).
        const Eigen::VectorXd xr = system->density.mean();
        Eigen::MatrixXd extra_cov =
            Eigen::MatrixXd::Zero(filter::SystemLocalisation::nx, filter::SystemLocalisation::nx);
        extra_cov(0, 0) = extra_cov(1, 1) = cfg.recovery_pos_std * cfg.recovery_pos_std;
        // Yaw is about the field z axis, which lands on the quaternion states as a rank-one block
        // rather than a single diagonal element.
        const Eigen::Vector4d j_yaw = filter::SystemLocalisation::attitudeTangentField(xr).col(2);
        extra_cov.block<4, 4>(filter::SystemLocalisation::iQuat, filter::SystemLocalisation::iQuat) =
            cfg.recovery_yaw_std * cfg.recovery_yaw_std * j_yaw * j_yaw.transpose();
        system->inflateCovariance(extra_cov);

        // A fall may have turned the robot around, which landmarks alone can never detect.
        // Re-seed the mirror so out-of-field evidence has something to switch to.
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

        // Roll, pitch and torso height come from the kinematic chain, unsearched.
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
                    // Rates start at zero; the body-rate measurements sharpen them quickly.
                    candidate << x, y, z0, rpy2quat(Eigen::Vector3d(roll0, pitch0, yaw)),
                        Eigen::Matrix<double, 9, 1>::Zero(), 0.0, 0.0;
                    probe.resetTo(GaussianInfo<double>::fromSqrtMoment(candidate, S_probe), sample.t);

                    filter::MeasurementFieldLandmarks measurement(sample.t, sample, Tbc, *map, probe, cfg.measurement);
                    if (measurement.numAssociated() < std::size_t(cfg.min_init_associations)) {
                        continue;
                    }
                    // The robust likelihood floors outliers at the clutter density, so this
                    // favours the pose explaining the most rays well.
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

        // Resolve the own-half/opponent-half symmetry. The mirror scores identically, so this
        // loses no fit; it only picks the side the kickoff rules allow. Own half is +x, the
        // field-frame convention the rest of the codebase hardcodes (own goal at +field_length/2).
        // Only true at kickoff -- see the out-of-field disambiguator for the rest of the game.
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

        // Planar world-to-field pose: keep (x, y, yaw), drop torso height and roll/pitch. The
        // field is a flat z=0 model and every consumer reasons about it on the ground plane.
        // Matches the NLopt convention: Hfw = Translation(x, y, 0) * Rz(yaw).
        const double yaw      = rot2rpy(Hfw_full.rotation()).z();
        Eigen::Isometry3d Hfw = Eigen::Isometry3d::Identity();
        Hfw.linear()          = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Hfw.translation()     = Eigen::Vector3d(Hfw_full.translation().x(), Hfw_full.translation().y(), 0.0);
        field->Hfw            = Hfw;

        // TODO: emit the feet in the field frame too, for passing and foot placement. Compose
        // Sensors.Htx[L_FOOT_BASE]/[R_FOOT_BASE] with the field pose.

        // Covariance of the reported (x, y, yaw). Yaw is not a state element but a direction in
        // the quaternion block, so it goes through row 2 of the attitude Jacobian.
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

        // Hypotheses as (x, y, yaw) particles; one component in single-hypothesis mode.
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
