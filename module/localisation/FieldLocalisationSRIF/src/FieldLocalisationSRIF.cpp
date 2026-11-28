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
#include "utility/math/euler.hpp"
#include "utility/math/quaternion.hpp"
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
    using measurement::MeasurementRest;
    using utility::gaussian_filtering::gaussian::GaussianInfo;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::math::quaternion::mat_to_wxyz;
    using utility::nusight::graph;
    using utility::support::Expression;

    /// @brief Single-channel 8-bit view of a camera frame, for FAST/ORB.
    /// @return False if the format is not one we can read, leaving `gray` untouched.
    static bool to_grayscale(const Image& image, cv::Mat& gray) {
        const int width  = int(image.dimensions.x());
        const int height = int(image.dimensions.y());
        uint8_t* data    = const_cast<uint8_t*>(image.data.data());
        switch (image.format) {
            case utility::vision::fourcc("BGR3"):
                cv::cvtColor(cv::Mat(height, width, CV_8UC3, data), gray, cv::COLOR_BGR2GRAY);
                return true;
            case utility::vision::FOURCC::RGB3:
                cv::cvtColor(cv::Mat(height, width, CV_8UC3, data), gray, cv::COLOR_RGB2GRAY);
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

    /// @brief Build a VisionSample (corner rays in {c}, ordered TL, TR, BR, BL) from YOLO boxes.
    ///        Class and confidence filtering happen in MeasurementFieldLandmarks.
    static filter::VisionSample build_vision_sample(double t, const BoundingBoxes& boxes) {
        filter::VisionSample sample;
        sample.t           = t;
        sample.video_frame = -1;
        sample.Hcw         = Eigen::Isometry3d(boxes.Hcw);

        for (const BoundingBox& box : boxes.bounding_boxes) {
            if (box.corners.size() != 4) {
                continue;
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

            // IMU pre-filtering, after VQF (Laidig & Seel, 2023)
            cfg.use_filtered_gravity = config["imu"]["use_filtered_gravity"].as<bool>();
            cfg.imu.tau_acc          = config["imu"]["tau_acc"].as<double>();
            cfg.use_rest_detection   = config["imu"]["use_rest_detection"].as<bool>();
            cfg.imu.rest_min_time    = config["imu"]["rest_min_time"].as<double>();
            cfg.rest_omega_sigma     = config["imu"]["rest_omega_sigma"].as<double>();
            cfg.rest_bias_sigma      = config["imu"]["rest_bias_sigma"].as<double>();
            imu.params               = cfg.imu;

            // How far vision is trusted
            cfg.measurement.sigma_angular          = config["measurement"]["sigma_angular"].as<double>();
            cfg.measurement.gate_angle             = config["measurement"]["gate_angle"].as<double>();
            cfg.measurement.min_confidence         = config["measurement"]["min_confidence"].as<double>();
            cfg.measurement.gate_yaw_scale         = config["measurement"]["gate_yaw_scale"].as<double>();
            cfg.measurement.gate_angle_max         = config["measurement"]["gate_angle_max"].as<double>();
            cfg.measurement.inlier_probability     = config["measurement"]["inlier_probability"].as<double>();
            cfg.measurement.confidence_reference   = config["measurement"]["confidence_reference"].as<double>();
            cfg.measurement.max_inlier_probability = config["measurement"]["max_inlier_probability"].as<double>();

            // How fast the belief may move
            cfg.process.sigma_vel       = config["process"]["sigma_vel"].as<double>();
            cfg.process.sigma_omega     = config["process"]["sigma_omega"].as<double>();
            cfg.process.sigma_pos_xy    = config["process"]["sigma_pos_xy"].as<double>();
            cfg.process.sigma_yaw       = config["process"]["sigma_yaw"].as<double>();
            cfg.process.sigma_gyro_bias = config["process"]["sigma_gyro_bias"].as<double>();
            cfg.process.sigma_cam_bias  = config["process"]["sigma_cam_bias"].as<double>();

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
            initialised        = false;
            was_upright        = true;
            gyro_bias_estimate = Eigen::Vector3d::Zero();
            // Its out-of-field map was built under the old belief
            side.reset();
            last_respawn_t = -std::numeric_limits<double>::infinity();
        });

        // Rolling odometry window; the shared Sync group guards it against the vision reaction
        on<Trigger<Sensors>, Sync<FieldLocalisationSRIF>, Priority::HIGH>().then([this](const Sensors& sensors) {
            if (!have_t0) {
                t0      = sensors.timestamp;
                have_t0 = true;
            }

            filter::SensorsSample s;
            s.t             = seconds(sensors.timestamp);
            s.Htw           = Eigen::Isometry3d(sensors.Htw);
            s.accelerometer = sensors.accelerometer;
            s.gyroscope     = sensors.gyroscope;

            // At the Sensors rate, so the filter sees every IMU sample and not only those paired with vision
            imu.update(s.t, s.gyroscope, s.accelerometer, gyro_bias_estimate);
            s.gravity        = imu.gravity();
            s.at_rest        = imu.at_rest();
            s.gyroscope_mean = imu.gyroscope_mean();

            // Non-finite when unavailable
            s.vBb = cfg.odometry_velocity_source == Config::OdometryVelocitySource::SENSORS_VTW
                        ? Eigen::Vector3d(s.Htw.linear() * Eigen::Vector3d(sensors.vTw))
                    : sensors_window.empty()
                        ? Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())
                        : filter::SystemLocalisation::body_velocity_from_odometry(sensors_window.back(),
                                                                                  s,
                                                                                  cfg.max_odometry_gap);

            // A reset or offset recapture shifts Htw in the plane without the torso moving. Compared
            // in the world frame so a fall's fast pitching is not mistaken for one.
            if (!sensors_window.empty()) {
                const Eigen::Isometry3d Twt_prev = sensors_window.back().Htw.inverse();
                const Eigen::Isometry3d Twt      = s.Htw.inverse();
                const double dxy         = (Twt.translation().head<2>() - Twt_prev.translation().head<2>()).norm();
                const Eigen::Matrix3d dR = Twt.linear() * Twt_prev.linear().transpose();
                const double dyaw        = std::abs(std::atan2(dR(1, 0), dR(0, 0)));
                if (dxy > cfg.odometry_jump_xy || dyaw > cfg.odometry_jump_angle) {
                    last_odometry_jump_t = s.t;
                    // A finite difference across the jump is not a velocity
                    s.vBb = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
                    log<INFO>("Odometry discontinuity of ",
                              dxy,
                              "m, ",
                              dyaw * 180.0 / M_PI,
                              "deg; holding off vision updates");
                }
            }

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
                if (map == nullptr || !have_t0) {
                    return;
                }

                const double t = seconds(boxes.timestamp);

                // Without a Stability message the robot is assumed upright
                const bool upright = stability == nullptr || *stability > Stability::FALLING;

                // Ahead of every early return: the first upright frame after a getup often has
                // no detections
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
                    system->set_posture(upright, upright ? 0.0 : t - fall_start_t);
                    was_upright = upright;
                }

                // Pair odometry with the capture time, since Hcw is at capture
                const filter::SensorsSample* paired = nearest_sensors(t);
                if (paired == nullptr) {
                    log<DEBUG>("No odometry sample near the vision frame; skipping");
                    if (initialised) {
                        system->predict_all(t);
                    }
                    return;
                }
                const Eigen::Isometry3d& Htw = paired->Htw;

                // Tbc = Htw * Hcw^{-1}
                const Eigen::Isometry3d Tbc = Htw * Eigen::Isometry3d(boxes.Hcw).inverse();

                const filter::VisionSample sample = build_vision_sample(t, boxes);

                // Body rates do not need detections
                if (initialised) {
                    if (paired->gyroscope.allFinite()) {
                        MeasurementGyroscope gyro(t, paired->gyroscope, cfg.gyroscope_sigma);
                        system->process(gyro);
                    }

                    // At rest the gyroscope reads its bias (VQF), the only yaw-rate bias information
                    // other than vision
                    if (cfg.use_rest_detection && paired->at_rest && paired->gyroscope_mean.allFinite()) {
                        MeasurementRest rest(t, paired->gyroscope_mean, cfg.rest_omega_sigma, cfg.rest_bias_sigma);
                        system->process(rest);
                    }

                    // Odometry velocity only while upright, otherwise a zero-velocity update
                    if (upright) {
                        if (paired->vBb.allFinite()) {
                            MeasurementBodyVelocity vel(t, paired->vBb, cfg.odometry_velocity_sigma);
                            system->process(vel);
                        }
                    }
                    else {
                        // Looser while falling or getting up, when the torso moves in place
                        const bool settled = stability != nullptr && *stability == Stability::FALLEN;
                        MeasurementBodyVelocity zupt =
                            MeasurementBodyVelocity::stationary(t, settled ? cfg.zupt_sigma : cfg.zupt_dynamic_sigma);
                        system->process(zupt);
                    }
                }

                // Tbc is unreliable across an odometry discontinuity. The field-frame belief is
                // unaffected, so Hfw is re-emitted against the newest Htw.
                if (near_odometry_jump(t)) {
                    if (initialised) {
                        system->predict_all(t);
                        emit_field(sensors_window.back().Htw, nullptr);
                    }
                    return;
                }

                // Advance the state and clock even without detections
                if (sample.detections.empty()) {
                    if (initialised) {
                        system->predict_all(t);
                    }
                    return;
                }

                // Initialise from a coarse grid search on the first usable frame
                if (!initialised) {
                    const Eigen::Isometry3d Twt = Htw.inverse();
                    Eigen::Matrix<double, 18, 1> eta0;
                    if (!solve_initial_pose(sample, Tbc, Twt, eta0)) {
                        log<DEBUG>("Initial pose solve did not associate enough landmarks yet");
                        return;
                    }

                    Eigen::MatrixXd S0 =
                        Eigen::MatrixXd::Zero(filter::SystemLocalisation::nx, filter::SystemLocalisation::nx);
                    S0.diagonal() = cfg.initial_sqrt_covariance;
                    const auto p0 = GaussianInfo<double>::from_sqrt_moment(Eigen::VectorXd(eta0), S0);

                    system         = std::make_unique<filter::SystemLocalisation>(p0);
                    system->params = cfg.process;
                    system->hyp    = cfg.hypothesis;
                    system->reset_to(p0, t);
                    if (cfg.use_hypothesis_bank) {
                        system->initialise_hypotheses();
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

                system->predict(t);

                // Applied first so the other updates see a belief on the unit sphere
                MeasurementQuaternionNorm quaternion_norm(t, cfg.quaternion_norm_sigma);
                system->process(quaternion_norm);

                // Through process() so the hypothesis bank reweights every component
                filter::MeasurementFieldLandmarks measurement(t, sample, Tbc, *map, *system, cfg.measurement);
                system->process(measurement);

                // Gravity only while the specific force is close to 1 g. The filtered reading averages
                // out the walking accelerations (see ImuPreFilter), so its magnitude is gated as well.
                const Eigen::Vector3d& specific_force =
                    cfg.use_filtered_gravity && paired->gravity.allFinite() ? paired->gravity : paired->accelerometer;
                if (cfg.use_gravity && specific_force.allFinite()) {
                    constexpr double standard_gravity = 9.80665;
                    const double a_mag                = specific_force.norm();
                    if (std::abs(a_mag - standard_gravity) < cfg.gravity_quasi_static_tolerance) {
                        MeasurementGravity gravity(t, specific_force, cfg.gravity_sigma);
                        system->process(gravity);
                    }
                }
                // Torso height assumes a foot on the ground
                if (cfg.use_kinematic_height && upright) {
                    const double height = Htw.inverse().translation().z();
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

                // Fed back to the IMU pre-filter's strapdown integration
                gyro_bias_estimate = filter::SystemLocalisation::gyro_bias(system->density.mean());

                emit_field(Htw, &measurement);
            });

        // Separate Single reaction so a slow frame drops corner detection, not a landmark update
        on<Trigger<Image>, Sync<FieldLocalisationSRIF>, Single>().then(
            "SRIF out-of-field side disambiguation",
            [this](const Image& image) { run_side_disambiguation(image); });
    }

    void FieldLocalisationSRIF::run_side_disambiguation(const Image& image) {
        if (!cfg.use_side_disambiguator || map == nullptr || !initialised || !have_t0) {
            return;
        }

        cv::Mat gray;
        if (!to_grayscale(image, gray)) {
            log<WARN>("Out-of-field disambiguation: unsupported image format ", utility::vision::fourcc(image.format));
            return;
        }

        // Built on the first frame, which carries the lens and image size
        if (side == nullptr) {
            side = std::make_unique<filter::SideDisambiguator>(
                image.lens,
                Eigen::Vector2d(double(image.dimensions.x()), double(image.dimensions.y())),
                map->dims);
        }

        const double t = seconds(image.timestamp);
        if (near_odometry_jump(t)) {
            return;
        }

        const filter::SensorsSample* paired = nearest_sensors(t);
        if (paired == nullptr) {
            log<DEBUG>("No odometry sample near the camera frame; skipping side disambiguation");
            return;
        }
        const Eigen::Isometry3d Tbc = paired->Htw * Eigen::Isometry3d(image.Hcw).inverse();

        // Camera pose in {f} at the mean and at its mirror
        const Eigen::VectorXd mean = system->density.mean();
        auto camera_pose           = [&Tbc](const Eigen::VectorXd& x) {
            return filter::SystemLocalisation::field_pose<double>(x) * Tbc
                   * Eigen::Isometry3d(filter::SystemLocalisation::camera_bias_rotation<double>(x));
        };
        const Eigen::VectorXd mirror = filter::SystemLocalisation::mirror_state(mean);

        const Eigen::MatrixXd P   = system->density.cov();
        const double pos_std      = std::sqrt(std::max(P(0, 0), P(1, 1)));
        const double yaw_std      = std::sqrt(filter::SystemLocalisation::yaw_variance(mean, P));
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
            // Bank mode: the evidence reweights the mixture instead of flipping the state
            system->add_side_log_evidence(result.side_delta);

            // Re-seed a pruned mirror when the evidence turns against the survivor
            if (result.flip_requested && system->num_hypotheses() == 1
                && t - last_respawn_t > side->options.flip_cooldown) {
                system->spawn_mirror();
                last_respawn_t = t;
                log<INFO>("Out-of-field evidence (llr ", result.llr, ") re-seeded the mirror hypothesis");
            }
        }
        else if (result.flip_requested) {
            // Single-hypothesis mode: mirror the belief outright
            system->reset_to(filter::SystemLocalisation::mirror_density(system->density), t);
            side->notify_flip_applied(t);
            const Eigen::VectorXd flipped = system->density.mean();
            log<INFO>("Out-of-field side flip (llr ",
                      result.llr,
                      ", assoc ",
                      result.n_associated,
                      "/",
                      result.n_associated_mirror,
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
                       double(result.n_associated),
                       double(result.n_associated_mirror)));
            emit(graph("SRIF/side landmarks", double(result.n_landmarks), double(result.n_candidates)));
        }
    }

    void FieldLocalisationSRIF::emit_out_of_field(const Image& image,
                                                  const filter::SideDisambiguator::FrameResult& result) {
        auto msg       = std::make_unique<OutOfFieldFeatures>();
        msg->id        = image.id;
        msg->timestamp = image.timestamp;
        msg->Hcw       = image.Hcw;

        // The status enums map by value
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
            feature.status = FeatureStatus(FeatureStatus::Value(result.feature_status[i]));
            msg->features.push_back(feature);
        }

        // Rays, so NUsight can re-project them and draw landmarks outside the image
        const Eigen::Vector2d dimensions(double(image.dimensions.x()), double(image.dimensions.y()));
        msg->landmarks.reserve(result.landmark_views.size());
        for (const filter::SideDisambiguator::LandmarkView& view : result.landmark_views) {
            OutOfFieldLandmark landmark;
            landmark.uPCc         = utility::vision::unproject_pixel(view.px, image.lens, dimensions);
            landmark.status       = LandmarkStatus(LandmarkStatus::Value(view.status));
            landmark.bearing_only = view.far;
            if (view.status == filter::SideDisambiguator::LANDMARK_ASSOCIATED) {
                landmark.uMatchCc = utility::vision::unproject_pixel(view.match_px, image.lens, dimensions);
            }
            msg->landmarks.push_back(landmark);
        }

        msg->llr             = result.llr;
        msg->side_delta      = result.side_delta;
        msg->landmark_count  = uint32_t(result.n_landmarks);
        msg->candidate_count = uint32_t(result.n_candidates);
        msg->map_frozen      = result.map_frozen;
        msg->flip_requested  = result.flip_requested;

        emit(msg);
    }

    void FieldLocalisationSRIF::apply_fall_recovery(double t) {
        // Keep the mean, widen the covariance. The wider yaw also reopens the association gate.
        const Eigen::VectorXd xr = system->density.mean();
        Eigen::MatrixXd extra_cov =
            Eigen::MatrixXd::Zero(filter::SystemLocalisation::nx, filter::SystemLocalisation::nx);
        extra_cov(0, 0) = extra_cov(1, 1) = cfg.recovery_pos_std * cfg.recovery_pos_std;
        // Yaw about field z is a rank-one block on the quaternion states
        const Eigen::Vector4d j_yaw = filter::SystemLocalisation::attitude_tangent_field(xr).col(2);
        extra_cov.block<4, 4>(filter::SystemLocalisation::i_quat, filter::SystemLocalisation::i_quat) =
            cfg.recovery_yaw_std * cfg.recovery_yaw_std * j_yaw * j_yaw.transpose();
        system->inflate_covariance(extra_cov);

        // A fall may have turned the robot around, which landmarks cannot detect
        if (cfg.use_hypothesis_bank && system->num_hypotheses() == 1) {
            system->spawn_mirror();
        }

        log<INFO>("Recovered after ",
                  t - fall_start_t,
                  "s not upright; inflated to sigma_xy ",
                  std::sqrt(system->density.cov()(0, 0)),
                  "m, sigma_yaw ",
                  std::sqrt(filter::SystemLocalisation::yaw_variance(system->density.mean(), system->density.cov()))
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
                                                   const Eigen::Isometry3d& Tbc,
                                                   const Eigen::Isometry3d& Twt,
                                                   Eigen::Matrix<double, 18, 1>& eta0) const {
        constexpr Eigen::Index nx = filter::SystemLocalisation::nx;

        // Roll, pitch and torso height come from the kinematic chain, unsearched.
        const Eigen::Vector3d rpy_torso = mat_to_rpy_intrinsic(Twt.linear());
        const double roll0              = rpy_torso.x();
        const double pitch0             = rpy_torso.y();
        const double z0                 = Twt.translation().z();

        const filter::FieldDimensions& dims = map->dims;
        const double half_length            = dims.field_length / 2.0 + dims.border_strip_min_width;
        const double half_width             = dims.field_width / 2.0 + dims.border_strip_min_width;

        // Probe system for association and likelihood scoring only
        const Eigen::MatrixXd S_probe = Eigen::MatrixXd::Identity(nx, nx) * 0.01;
        filter::SystemLocalisation probe(GaussianInfo<double>::from_sqrt_moment(Eigen::VectorXd::Zero(nx), S_probe));

        double best_score        = -std::numeric_limits<double>::infinity();
        std::size_t best_assoc   = 0;
        Eigen::VectorXd best_eta = Eigen::VectorXd::Zero(nx);

        for (double x = -half_length; x <= half_length; x += cfg.grid_step_xy) {
            for (double y = -half_width; y <= half_width; y += cfg.grid_step_xy) {
                for (double yaw = -M_PI; yaw < M_PI; yaw += cfg.grid_step_yaw) {
                    Eigen::VectorXd candidate(nx);
                    candidate << x, y, z0, mat_to_wxyz(rpy_intrinsic_to_mat(Eigen::Vector3d(roll0, pitch0, yaw))),
                        Eigen::Matrix<double, 9, 1>::Zero(), 0.0, 0.0;
                    probe.reset_to(GaussianInfo<double>::from_sqrt_moment(candidate, S_probe), sample.t);

                    filter::MeasurementFieldLandmarks measurement(sample.t, sample, Tbc, *map, probe, cfg.measurement);
                    if (measurement.num_associated() < std::size_t(cfg.min_init_associations)) {
                        continue;
                    }
                    const double score = measurement.log_likelihood(candidate, probe);
                    if (score > best_score) {
                        best_score = score;
                        best_assoc = measurement.num_associated();
                        best_eta   = candidate;
                    }
                }
            }
        }

        if (best_assoc < std::size_t(cfg.min_init_associations)) {
            return false;
        }

        // The mirror scores identically; pick our own half (+x, own goal at +field_length/2)
        if (best_eta(0) < 0.0) {
            best_eta = filter::SystemLocalisation::mirror_state(best_eta);
        }

        eta0 = best_eta;
        return true;
    }

    void FieldLocalisationSRIF::emit_field(const Eigen::Isometry3d& Htw,
                                           const filter::MeasurementFieldLandmarks* measurement) {
        const Eigen::VectorXd mean = system->density.mean();

        // Hfw = Tfb(mean) * Htw
        const Eigen::Isometry3d Tfb      = filter::SystemLocalisation::field_pose<double>(mean);
        const Eigen::Isometry3d Hfw_full = Tfb * Htw;
        auto field                       = std::make_unique<Field>();

        // Planar, as FieldLocalisationNLopt: Hfw = Translation(x, y, 0) * Rz(yaw)
        const double yaw      = mat_to_rpy_intrinsic(Hfw_full.rotation()).z();
        Eigen::Isometry3d Hfw = Eigen::Isometry3d::Identity();
        Hfw.linear()          = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Hfw.translation()     = Eigen::Vector3d(Hfw_full.translation().x(), Hfw_full.translation().y(), 0.0);
        field->Hfw            = Hfw;
        // TODO: also emit the feet in {f}, for passing and foot placement

        // Covariance of (x, y, yaw); yaw goes through row 2 of the attitude Jacobian
        const Eigen::MatrixXd P           = system->density.cov();
        constexpr Eigen::Index iq         = filter::SystemLocalisation::i_quat;
        const Eigen::RowVector4d g_yaw    = filter::SystemLocalisation::attitude_jacobian(mean).row(2);
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
        field->localised =
            std::sqrt(P(0, 0) + P(1, 1)) < cfg.localised_pos_std && std::sqrt(var_yaw) < cfg.localised_yaw_std;

        // Hypotheses as (x, y, yaw) particles; one component in single-hypothesis mode.
        if (system->num_hypotheses() > 1) {
            for (const GaussianInfo<double>& component : system->hypotheses()) {
                const Eigen::VectorXd m = component.mean();
                field->particles.emplace_back(m(0), m(1), filter::SystemLocalisation::heading(m));
            }
        }
        else {
            field->particles.emplace_back(mean(0), mean(1), filter::SystemLocalisation::heading(mean));
        }

        // Cost: mean angular residual of the associated rays [rad]
        double cost = 0.0;
        if (measurement != nullptr && measurement->num_associated() > 0) {
            const Eigen::Matrix<double, 3, Eigen::Dynamic>& measured = measurement->measured_rays();
            const Eigen::Matrix<double, 3, Eigen::Dynamic> predicted = measurement->predict_rays<double>(mean);
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
                emit(graph("SRIF associations", double(measurement->num_associated())));
            }
        }
    }

}  // namespace module::localisation
