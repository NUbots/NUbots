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
#include "ExtrinsicsCalibration.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <fmt/format.h>
#include <fstream>
#include <limits>
#include <nlopt.hpp>
#include <random>
#include <tinyrobotics/kinematics.hpp>
#include <tinyrobotics/parser.hpp>
#include <tuple>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/input/Sensors.hpp"
#include "message/planning/LookAround.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/FallRecovery.hpp"

#include "utility/algorithm/assignment.hpp"
#include "utility/input/FrameID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"
#include "utility/platform/aliases.hpp"
#include "utility/support/network.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::tools {

    using extension::Configuration;
    using extension::behaviour::Task;

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;
    using message::planning::LookAround;
    using message::skill::Walk;
    using message::strategy::FallRecovery;
    using message::support::FieldDescription;
    using message::vision::FieldIntersection;
    using message::vision::FieldIntersections;

    using utility::input::FrameID;
    using utility::input::ServoID;
    using utility::localisation::Landmark;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::support::Expression;

    ExtrinsicsCalibration::ExtrinsicsCalibration(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("ExtrinsicsCalibration.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.urdf_path      = config["urdf_path"].as<std::string>();
            cfg.camera         = config["camera"].as<std::string>();
            cfg.is_left_camera = cfg.camera == "Left";
            cfg.min_samples    = config["min_samples"].as<size_t>();

            cfg.max_association_distance = config["max_association_distance"].as<double>();
            cfg.min_head_pose_change     = config["min_head_pose_change"].as<Expression>();
            cfg.rotation_bounds          = Eigen::Vector3d(config["rotation_bounds"].as<Expression>());
            cfg.translation_bounds       = Eigen::Vector3d(config["translation_bounds"].as<Expression>());
            cfg.split_ratio              = config["validation"]["split_ratio"].as<double>();
            cfg.split_seed               = config["validation"]["seed"].as<uint64_t>();

            cfg.xtol_rel           = config["opt"]["xtol_rel"].as<double>();
            cfg.ftol_rel           = config["opt"]["ftol_rel"].as<double>();
            cfg.maxeval            = config["opt"]["maxeval"].as<size_t>();
            cfg.max_icp_iterations = config["max_icp_iterations"].as<size_t>();

            // Compute the URDF nominal Hpc (head-pitch {p} from camera {c}) using forward kinematics, the same
            // way the Camera module does. This is the centre of the optimisation search box and the seed the
            // camera configs should hold before calibration.
            auto nugus_model  = tinyrobotics::import_urdf<double, 20>(cfg.urdf_path);
            auto camera_frame = cfg.is_left_camera ? std::string("left_camera") : std::string("right_camera");
            auto Hpc          = tinyrobotics::forward_kinematics<double, 20>(nugus_model,
                                                                    nugus_model.home_configuration(),
                                                                    camera_frame,
                                                                    std::string("head"));
            Hpc_base          = Eigen::Isometry3d(Hpc.matrix());

            // Decompose the nominal into the 6-DOF pose [roll, pitch, yaw, tx, ty, tz].
            nominal_pose.head<3>() = mat_to_rpy_intrinsic(Hpc_base.rotation());
            nominal_pose.tail<3>() = Hpc_base.translation();
            log<INFO>(fmt::format(
                "URDF nominal Hpc seed ({} camera): translation = [{:.6f}, {:.6f}, {:.6f}] m, "
                "rpy = [{:.6f}, {:.6f}, {:.6f}] rad = [{:.3f}, {:.3f}, {:.3f}] deg",
                cfg.camera,
                nominal_pose(3),
                nominal_pose(4),
                nominal_pose(5),
                nominal_pose(0),
                nominal_pose(1),
                nominal_pose(2),
                nominal_pose(0) * 180.0 / M_PI,
                nominal_pose(1) * 180.0 / M_PI,
                nominal_pose(2) * 180.0 / M_PI));

            // Warm-start the optimisation from the robot's current config Hpc; fall back to the URDF nominal.
            std::string hostname   = utility::support::get_hostname();
            std::string robot_name = utility::platform::get_robot_alias(hostname);
            camera_config_path     = fmt::format("config/{}/Cameras/{}.yaml", robot_name, cfg.camera);

            current_pose = nominal_pose;
            try {
                YAML::Node cam_cfg          = YAML::LoadFile(camera_config_path);
                Eigen::Vector3d translation = Eigen::Vector3d(cam_cfg["translation"].as<Expression>());
                Eigen::Vector3d rpy         = Eigen::Vector3d(cam_cfg["rpy"].as<Expression>());
                current_pose.head<3>()      = rpy;
                current_pose.tail<3>()      = translation;
                log<INFO>(fmt::format(
                    "Calibrating {} camera for {}. Warm-start pose: translation = [{:.4f}, {:.4f}, {:.4f}] m, "
                    "rpy = [{:.3f}, {:.3f}, {:.3f}] deg",
                    cfg.camera,
                    robot_name,
                    current_pose(3),
                    current_pose(4),
                    current_pose(5),
                    current_pose(0) * 180.0 / M_PI,
                    current_pose(1) * 180.0 / M_PI,
                    current_pose(2) * 180.0 / M_PI));
            }
            catch (const std::exception& e) {
                log<ERROR>(fmt::format("Failed to read camera config '{}': {} (warm-starting from URDF nominal)",
                                       camera_config_path,
                                       e.what()));
            }
        });

        // Generate the ground-truth landmarks once the field description is known
        on<Startup, Trigger<FieldDescription>>().then("Setup field landmarks", [this](const FieldDescription& fd) {
            landmarks = utility::localisation::setup_field_landmarks(fd);
            log<INFO>(fmt::format("Loaded {} ground-truth field landmarks", landmarks.size()));
        });

        // Stand the robot up so the head servos are powered and it holds a stable posture while calibrating.
        on<Startup>().then("Calibration stance", [this] {
            // These seed messages let modules that depend on a Stability / WalkState run.
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));
            // Always try to recover from a fall, and stand still (a zero-velocity walk).
            emit<Task>(std::make_unique<FallRecovery>(), 1);
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()));
        });

        // Main calibration loop: collect associated detections while the robot is static at the field centre,
        // then run the optimisation once enough data has been gathered.
        on<Trigger<FieldIntersections>, With<Sensors>, With<FieldDescription>, Single>().then(
            "ExtrinsicsCalibration",
            [this](const FieldIntersections& field_intersections, const Sensors& sensors, const FieldDescription&) {
                // Stop collecting / optimising once we are done. We also stop requesting the head sweep so the head
                // holds its last position while the robot keeps standing (rather than continuing to scan).
                if (calibrated) {
                    return;
                }

                // Sweep the head automatically via the PlanLook provider, which steps through the (yaw, pitch) grid
                // configured in the role-specific PlanLook.yaml (config/bin/extrinsicscalibration/PlanLook.yaml) so
                // the field landmarks are seen across the whole image. Re-emitted each frame (FieldIntersections is
                // published every frame) so PlanLook keeps advancing through the grid.
                emit<Task>(std::make_unique<LookAround>(), 1);

                // Only gather a sample once the head has swept far enough since the last capture, so the data
                // spans distinct viewpoints (the head sweeps automatically) rather than over-sampling one pose.
                const double head_yaw   = sensors.servo[ServoID::HEAD_YAW].present_position;
                const double head_pitch = sensors.servo[ServoID::HEAD_PITCH].present_position;
                if (!frames.empty() && std::abs(head_yaw - last_head_yaw) < cfg.min_head_pose_change
                    && std::abs(head_pitch - last_head_pitch) < cfg.min_head_pose_change) {
                    return;
                }
                last_head_yaw   = head_yaw;
                last_head_pitch = head_pitch;

                // --- Build the known transforms for this frame ---
                // Offset-free world {w} from head-pitch {p}, computed the same way as the Camera module
                Eigen::Isometry3d Htw(sensors.Htw);
                Eigen::Isometry3d Htp(sensors.Htx[FrameID::HEAD_PITCH]);
                // Known field {f} from world {w}, synthesised from the placement assumption: the torso is at the
                // field centre (x = y = 0) facing the goal, with roll/pitch/height taken from sensors.
                Eigen::Isometry3d Hwt = Htw.inverse();
                Eigen::Isometry3d Hwp = Hwt * Htp;

                Eigen::Vector3d rpy   = mat_to_rpy_intrinsic(Hwt.rotation());
                Eigen::Isometry3d Hft = Eigen::Isometry3d::Identity();
                Hft.linear()          = rpy_intrinsic_to_mat(Eigen::Vector3d(rpy.x(), rpy.y(), 0.0));
                Hft.translation().z() = Hwt.translation().z();
                Eigen::Isometry3d Hfw = Hft * Htw;

                // Store this frame's raw detections. Association is deferred to the ICP loop so it can be
                // re-evaluated as the offsets are refined.
                collect(field_intersections, Hwp, Hfw);
                log<INFO>(fmt::format("Collected {}/{} detections across {} frames",
                                      collected_detections,
                                      cfg.min_samples,
                                      frames.size()));
                if (collected_detections >= cfg.min_samples) {
                    partition_observations();
                    run_calibration();
                }
            });
    }

    Eigen::Isometry3d ExtrinsicsCalibration::pose_to_Hpc(const Vector6d& pose) const {
        // pose = [roll, pitch, yaw, tx, ty, tz]; rotation uses the same ZYX-intrinsic rpy convention as the
        // Camera module (rpy_intrinsic_to_mat), translation is the camera origin in head-pitch {p} space.
        Eigen::Isometry3d Hpc = Eigen::Isometry3d::Identity();
        Hpc.linear()          = rpy_intrinsic_to_mat(Eigen::Vector3d(pose.head<3>()));
        Hpc.translation()     = pose.tail<3>();
        return Hpc;
    }

    Eigen::Vector3d ExtrinsicsCalibration::project_to_field(const Vector6d& pose, const Sample& sample) const {
        // Build the candidate camera {c} to head-pitch {p} transform directly from the 6-DOF pose.
        const Eigen::Isometry3d Hpc = pose_to_Hpc(pose);

        // World {w} from camera {c}
        const Eigen::Isometry3d Hwc = sample.Hwp * Hpc;

        // Project the detection ray onto the ground plane (z = 0) in world space, then transform to field space
        const Eigen::Vector3d uICw   = Hwc.rotation() * sample.uICc;
        const Eigen::Vector3d origin = Hwc.translation();
        const Eigen::Vector3d rIWw   = uICw * std::abs(origin.z() / uICw.z()) + origin;
        return sample.Hfw * rIWw;
    }

    void ExtrinsicsCalibration::collect(const FieldIntersections& field_intersections,
                                        const Eigen::Isometry3d& Hwp,
                                        const Eigen::Isometry3d& Hfw) {
        if (field_intersections.intersections.empty()) {
            return;
        }

        // Recover each detection's offset-independent camera-frame ray (the pixel ray with the live offsets
        // divided back out) and stash the raw frame for later (re-)association.
        const Eigen::Isometry3d Hcw(field_intersections.Hcw);
        Frame frame;
        frame.Hwp = Hwp;
        frame.Hfw = Hfw;
        frame.observations.reserve(field_intersections.intersections.size());
        for (const auto& intersection : field_intersections.intersections) {
            Observation obs;
            obs.uICc = (Hcw * intersection.rIWw).normalized();
            obs.type = intersection.type;
            frame.observations.push_back(obs);
        }

        collected_detections += frame.observations.size();
        frames.push_back(std::move(frame));
    }

    std::vector<int> ExtrinsicsCalibration::associate(const Vector6d& pose, bool validation) {
        // Cross-type pairings are blocked with a large finite cost to avoid overflow in the assignment solver.
        constexpr double MAX_ANTIOVERFLOW_COST = 1e9;

        samples.clear();
        std::vector<int> signature;
        signature.reserve(collected_detections);

        // Associate per frame so the Hungarian assignment stays one-to-one between a frame's detections and the
        // landmarks (a single frame never sees the same landmark twice).
        for (const auto& frame : frames) {
            // Restrict to the detections in the requested split (training or held-out validation).
            std::vector<size_t> idx;
            idx.reserve(frame.observations.size());
            for (size_t d = 0; d < frame.observations.size(); ++d) {
                if (frame.observations[d].validation == validation) {
                    idx.push_back(d);
                }
            }
            const size_t n = idx.size();
            if (n == 0) {
                continue;
            }

            // Project each detection to field space at the given pose, then build the cost matrix.
            Eigen::MatrixXd cost_matrix(n, landmarks.size());
            for (size_t i = 0; i < n; ++i) {
                Sample probe;
                probe.uICc                 = frame.observations[idx[i]].uICc;
                probe.Hwp                  = frame.Hwp;
                probe.Hfw                  = frame.Hfw;
                const Eigen::Vector3d rIFf = project_to_field(pose, probe);
                for (size_t l = 0; l < landmarks.size(); ++l) {
                    cost_matrix(i, l) = (landmarks[l].type == frame.observations[idx[i]].type)
                                            ? (landmarks[l].rLFf - rIFf).norm()
                                            : MAX_ANTIOVERFLOW_COST;
                }
            }

            const auto assignment = utility::algorithm::determine_assignment(cost_matrix);
            for (size_t i = 0; i < n; ++i) {
                int assigned_landmark = -1;
                const auto it         = assignment.find(static_cast<int>(i));
                if (it != assignment.end()) {
                    const int l = it->second;
                    // Gate on the field-space distance to reject ambiguous / wrong-type pairings.
                    if (cost_matrix(i, l) < cfg.max_association_distance) {
                        Sample s = Sample{};
                        s.uICc   = frame.observations[idx[i]].uICc;
                        s.Hwp    = frame.Hwp;
                        s.Hfw    = frame.Hfw;
                        s.rLFf   = landmarks[l].rLFf;
                        samples.push_back(s);
                        assigned_landmark = l;
                    }
                }
                signature.push_back(assigned_landmark);
            }
        }

        return signature;
    }

    void ExtrinsicsCalibration::partition_observations() {
        // Deterministically (seeded) tag each detection as validation with probability cfg.split_ratio. Done
        // per-detection so the held-out set is independent of the head-sweep order used to gather the data.
        std::mt19937_64 rng(cfg.split_seed);
        std::bernoulli_distribution is_val(cfg.split_ratio);
        size_t n_val = 0;
        for (auto& frame : frames) {
            for (auto& obs : frame.observations) {
                obs.validation = is_val(rng);
                n_val += obs.validation ? 1 : 0;
            }
        }
        log<INFO>(fmt::format("Train/validation split (seed {}): {} validation / {} total detections (~{:.0f}%)",
                              cfg.split_seed,
                              n_val,
                              collected_detections,
                              100.0 * cfg.split_ratio));
    }

    void ExtrinsicsCalibration::run_calibration() {
        // Capture the warm-start pose (the config values the run began with) so the optimised fit can be
        // compared against the starting fit once the optimisation finishes.
        const Vector6d warm_start_pose = current_pose;
        std::vector<int> previous_signature;
        Vector6d pose        = current_pose;
        double cost          = 0.0;
        nlopt::result result = nlopt::FAILURE;

        for (size_t iter = 0; iter < cfg.max_icp_iterations; ++iter) {
            // Reassociate the TRAINING detections at the current best pose.
            const std::vector<int> signature = associate(current_pose, /*validation=*/false);
            if (samples.empty()) {
                log<ERROR>("No training detections could be associated with any landmark; aborting calibration.");
                return;
            }

            // Optimise the pose with the associations held fixed.
            std::tie(pose, cost, result) = optimise();
            log<INFO>(fmt::format(
                "ICP iter {}: {} train samples, rpy (deg) = [{:.3f}, {:.3f}, {:.3f}], t (m) = "
                "[{:.4f}, {:.4f}, {:.4f}], cost = {:.6f}",
                iter + 1,
                samples.size(),
                pose(0) * 180.0 / M_PI,
                pose(1) * 180.0 / M_PI,
                pose(2) * 180.0 / M_PI,
                pose(3),
                pose(4),
                pose(5),
                cost));

            if (result < 0) {
                log<ERROR>(fmt::format("Optimisation failed: {}",
                                       ::nlopt_result_to_string(static_cast<nlopt_result>(result))));
                return;
            }

            // Re-centre the next association / optimisation on the refined pose.
            current_pose = pose;

            // Converged once the refined pose reproduces the same associations
            if (signature == previous_signature) {
                log<INFO>(fmt::format("ICP converged after {} iteration(s): associations stable.", iter + 1));
                calibrated = true;
                break;
            }
            previous_signature = signature;
        }

        // Compare the warm-start vs optimised fit on the training AND held-out validation sets
        // Each pose is scored under its OWN associations. RMS is the per-sample re-projection error in metres. A
        // lower OPTIMISED validation RMS means the extra DOF generalise rather than overfitting the gather setup.
        // If the warm start fits validation better, the optimiser is chasing noise / initial placement is wrong. NOTE:
        // associate() rebuilds the shared `samples` buffer, but the ICP loop is finished so clobbering it here is
        // harmless.
        const auto evaluate = [this](const Vector6d& p, bool validation) {
            associate(p, validation);
            const size_t n   = samples.size();
            const double c   = total_cost(p);
            const double rms = n > 0 ? std::sqrt(c / static_cast<double>(n)) : 0.0;
            return std::make_tuple(n, rms);
        };
        const auto log_fit = [this, &evaluate](const std::string& label, const Vector6d& p) {
            const auto [train_n, train_rms] = evaluate(p, /*validation=*/false);
            const auto [val_n, val_rms]     = evaluate(p, /*validation=*/true);
            log<INFO>(fmt::format(
                "{} | rpy (deg) = [{:.3f}, {:.3f}, {:.3f}], t (m) = [{:.4f}, {:.4f}, {:.4f}] | "
                "train: {} samples, RMS = {:.4f} m | val: {} samples, RMS = {:.4f} m",
                label,
                p(0) * 180.0 / M_PI,
                p(1) * 180.0 / M_PI,
                p(2) * 180.0 / M_PI,
                p(3),
                p(4),
                p(5),
                train_n,
                train_rms,
                val_n,
                val_rms));
        };
        log_fit("Warm start", warm_start_pose);
        log_fit("Optimised ", current_pose);

        // How far the translation moved from the URDF nominal (should be cm-scale).
        const Eigen::Vector3d dt_cm = (current_pose.tail<3>() - nominal_pose.tail<3>()) * 100.0;
        log<INFO>(fmt::format("Translation delta from URDF nominal: [{:.2f}, {:.2f}, {:.2f}] cm",
                              dt_cm.x(),
                              dt_cm.y(),
                              dt_cm.z()));

        if (calibrated) {
            write_extrinsics(current_pose);
            log<INFO>("New extrinsics written to config. Calibration complete.");
        }
    }

    double ExtrinsicsCalibration::total_cost(const Vector6d& pose) const {
        double cost = 0.0;
        for (const auto& sample : samples) {
            cost += (project_to_field(pose, sample) - sample.rLFf).squaredNorm();
        }
        return cost;
    }

    double ExtrinsicsCalibration::objective(const std::vector<double>& x, std::vector<double>& grad, void* data) {
        (void) grad;  // BOBYQA is derivative-free
        auto* self = static_cast<ExtrinsicsCalibration*>(data);
        // Optimisation variables are the absolute pose params [roll, pitch, yaw, tx, ty, tz]
        Vector6d pose;
        pose << x[0], x[1], x[2], x[3], x[4], x[5];
        return self->total_cost(pose);
    }

    std::tuple<ExtrinsicsCalibration::Vector6d, double, nlopt::result> ExtrinsicsCalibration::optimise() {
        nlopt::opt opt(nlopt::LN_BOBYQA, n_params);
        opt.set_min_objective(ExtrinsicsCalibration::objective, this);
        opt.set_xtol_rel(cfg.xtol_rel);
        opt.set_ftol_rel(cfg.ftol_rel);
        opt.set_maxeval(static_cast<int>(cfg.maxeval));

        // Box bounds centred on the URDF nominal pose: rotation half-widths then translation half-widths.
        Vector6d half;
        half << cfg.rotation_bounds, cfg.translation_bounds;
        std::vector<double> lb(n_params), ub(n_params), x(n_params);
        for (unsigned int i = 0; i < n_params; ++i) {
            lb[i] = nominal_pose(i) - half(i);
            ub[i] = nominal_pose(i) + half(i);
            // Warm-start from the current best pose, clamped into the search box.
            x[i] = std::min(std::max(current_pose(i), lb[i]), ub[i]);
        }
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);

        double final_cost    = 0.0;
        nlopt::result result = nlopt::FAILURE;
        try {
            result = opt.optimize(x, final_cost);
        }
        catch (const std::exception& e) {
            log<ERROR>(fmt::format("Optimisation failed: {}", e.what()));
        }

        Vector6d pose;
        pose << x[0], x[1], x[2], x[3], x[4], x[5];
        return {pose, final_cost, result};
    }

    void ExtrinsicsCalibration::write_extrinsics(const Vector6d& pose) {
        try {
            YAML::Node cam_cfg = YAML::LoadFile(camera_config_path);

            // Drop any legacy offset keys so the migrated schema stays clean.
            cam_cfg.remove("roll_offset");
            cam_cfg.remove("pitch_offset");
            cam_cfg.remove("yaw_offset");

            // translation [m] as plain numbers
            YAML::Node translation(YAML::NodeType::Sequence);
            translation.SetStyle(YAML::EmitterStyle::Flow);
            translation.push_back(pose(3));
            translation.push_back(pose(4));
            translation.push_back(pose(5));
            cam_cfg["translation"] = translation;

            // rpy [rad] written as human-readable "<degrees> * pi / 180" expression strings
            YAML::Node rpy(YAML::NodeType::Sequence);
            rpy.SetStyle(YAML::EmitterStyle::Flow);
            rpy.push_back(fmt::format("{} * pi / 180", pose(0) * 180.0 / M_PI));
            rpy.push_back(fmt::format("{} * pi / 180", pose(1) * 180.0 / M_PI));
            rpy.push_back(fmt::format("{} * pi / 180", pose(2) * 180.0 / M_PI));
            cam_cfg["rpy"] = rpy;

            std::ofstream file(camera_config_path);
            file << cam_cfg;
            log<INFO>(fmt::format("Wrote optimised extrinsics to {}", camera_config_path));
        }
        catch (const std::exception& e) {
            log<ERROR>(fmt::format("Failed to write camera config '{}': {}", camera_config_path, e.what()));
        }
    }

}  // namespace module::tools
