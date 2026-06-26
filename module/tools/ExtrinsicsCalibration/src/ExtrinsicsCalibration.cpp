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
#include <cmath>
#include <fmt/format.h>
#include <fstream>
#include <limits>
#include <nlopt.hpp>
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
            cfg.offset_bounds            = Eigen::Vector3d(config["offset_bounds"].as<Expression>());

            cfg.xtol_rel           = config["opt"]["xtol_rel"].as<double>();
            cfg.ftol_rel           = config["opt"]["ftol_rel"].as<double>();
            cfg.maxeval            = config["opt"]["maxeval"].as<size_t>();
            cfg.max_icp_iterations = config["max_icp_iterations"].as<size_t>();

            // Compute the offset-free base Hpc (head-pitch {p} from camera {c}) using forward kinematics, the
            // same way the Camera module does before applying the extrinsic offsets.
            auto nugus_model  = tinyrobotics::import_urdf<double, 20>(cfg.urdf_path);
            auto camera_frame = cfg.is_left_camera ? std::string("left_camera") : std::string("right_camera");
            auto Hpc          = tinyrobotics::forward_kinematics<double, 20>(nugus_model,
                                                                             nugus_model.home_configuration(),
                                                                             camera_frame,
                                                                             std::string("head"));
            Hpc_base          = Eigen::Isometry3d(Hpc.matrix());

            // Read the robot's current extrinsic offsets, which form the initial guess for the optimisation
            std::string hostname   = utility::support::get_hostname();
            std::string robot_name = utility::platform::get_robot_alias(hostname);
            camera_config_path     = fmt::format("config/{}/Cameras/{}.yaml", robot_name, cfg.camera);

            try {
                YAML::Node cam_cfg = YAML::LoadFile(camera_config_path);
                current_offsets    = Eigen::Vector3d(cam_cfg["roll_offset"].as<Expression>(),
                                                     cam_cfg["pitch_offset"].as<Expression>(),
                                                     cam_cfg["yaw_offset"].as<Expression>());
                log<INFO>(fmt::format(
                    "Calibrating {} camera for {}. Initial offsets (deg): roll = {:.3f}, pitch = {:.3f}, yaw = {:.3f}",
                    cfg.camera,
                    robot_name,
                    current_offsets.x() * 180.0 / M_PI,
                    current_offsets.y() * 180.0 / M_PI,
                    current_offsets.z() * 180.0 / M_PI));
            }
            catch (const std::exception& e) {
                log<ERROR>(fmt::format("Failed to read camera config '{}': {}", camera_config_path, e.what()));
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
                const double neck_yaw   = sensors.servo[ServoID::NECK_YAW].present_position;
                const double head_pitch = sensors.servo[ServoID::HEAD_PITCH].present_position;
                if (!frames.empty() && std::abs(neck_yaw - last_head_yaw) < cfg.min_head_pose_change
                    && std::abs(head_pitch - last_head_pitch) < cfg.min_head_pose_change) {
                    return;
                }
                last_head_yaw   = neck_yaw;
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
                    run_calibration();
                }
            });
    }

    Eigen::Matrix3d ExtrinsicsCalibration::offset_rotation(const Eigen::Vector3d& offsets) const {
        // offsets = (roll, pitch, yaw). Matches the convention used in the Camera module:
        // R = Rx(yaw) * Rz(pitch) * Ry(roll)
        return (Eigen::AngleAxisd(offsets.z(), Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(offsets.y(), Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(offsets.x(), Eigen::Vector3d::UnitY()))
            .toRotationMatrix();
    }

    Eigen::Vector3d ExtrinsicsCalibration::project_to_field(const Eigen::Vector3d& offsets,
                                                            const Sample& sample) const {
        // Apply the candidate offset rotation to the base camera transform (R_offset * Hpc_base)
        const Eigen::Matrix3d R = offset_rotation(offsets);
        Eigen::Isometry3d Hpc   = Eigen::Isometry3d::Identity();
        Hpc.linear()            = R * Hpc_base.linear();
        Hpc.translation()       = R * Hpc_base.translation();

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

    std::vector<int> ExtrinsicsCalibration::associate(const Eigen::Vector3d& offsets) {
        // Cross-type pairings are blocked with a large finite cost to avoid overflow in the assignment solver.
        constexpr double MAX_ANTIOVERFLOW_COST = 1e9;

        samples.clear();
        std::vector<int> signature;
        signature.reserve(collected_detections);

        // Associate per frame so the Hungarian assignment stays one-to-one between a frame's detections and the
        // landmarks (a single frame never sees the same landmark twice).
        for (const auto& frame : frames) {
            const size_t n = frame.observations.size();
            if (n == 0) {
                continue;
            }

            // Project each detection to field space at the given offsets, then build the cost matrix.
            Eigen::MatrixXd cost_matrix(n, landmarks.size());
            for (size_t d = 0; d < n; ++d) {
                Sample probe;
                probe.uICc                 = frame.observations[d].uICc;
                probe.Hwp                  = frame.Hwp;
                probe.Hfw                  = frame.Hfw;
                const Eigen::Vector3d rIFf = project_to_field(offsets, probe);
                for (size_t l = 0; l < landmarks.size(); ++l) {
                    cost_matrix(d, l) = (landmarks[l].type == frame.observations[d].type)
                                            ? (landmarks[l].rLFf - rIFf).norm()
                                            : MAX_ANTIOVERFLOW_COST;
                }
            }

            const auto assignment = utility::algorithm::determine_assignment(cost_matrix);
            for (size_t d = 0; d < n; ++d) {
                int assigned_landmark = -1;
                const auto it         = assignment.find(static_cast<int>(d));
                if (it != assignment.end()) {
                    const int l = it->second;
                    // Gate on the field-space distance to reject ambiguous / wrong-type pairings.
                    if (cost_matrix(d, l) < cfg.max_association_distance) {
                        Sample s = Sample{};
                        s.uICc   = frame.observations[d].uICc;
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

    void ExtrinsicsCalibration::run_calibration() {
        // Capture the warm-start offsets (the config values the run began with) so the optimised fit can be
        // compared against the starting fit once the optimisation finishes.
        const Eigen::Vector3d warm_start_offsets = current_offsets;
        std::vector<int> previous_signature;
        Eigen::Vector3d offsets = current_offsets;
        double cost             = 0.0;
        nlopt::result result    = nlopt::FAILURE;

        for (size_t iter = 0; iter < cfg.max_icp_iterations; ++iter) {
            // Reassociate every collected detection at the current best offsets
            const std::vector<int> signature = associate(current_offsets);
            if (samples.empty()) {
                log<ERROR>("No detections could be associated with any landmark; aborting calibration.");
                return;
            }

            // Optimise the offsets with the associations held fixed.
            std::tie(offsets, cost, result) = optimise();
            log<INFO>(fmt::format(
                "ICP iter {}: {} samples, offsets (deg) roll = {:.3f}, pitch = {:.3f}, yaw = {:.3f}, cost = {:.6f}",
                iter + 1,
                samples.size(),
                offsets.x() * 180.0 / M_PI,
                offsets.y() * 180.0 / M_PI,
                offsets.z() * 180.0 / M_PI,
                cost));

            if (result < 0) {
                log<ERROR>(fmt::format("Optimisation failed: {}",
                                       ::nlopt_result_to_string(static_cast<nlopt_result>(result))));
                return;
            }

            // Re-centre the next association / optimisation on the refined offsets.
            current_offsets = offsets;

            // Converged once the refined offsets reproduce the same associations
            if (signature == previous_signature) {
                log<INFO>(fmt::format("ICP converged after {} iteration(s): associations stable.", iter + 1));
                calibrated = true;
                break;
            }
            previous_signature = signature;
        }

        log<INFO>(fmt::format(
            "Final optimised offsets (deg): roll = {:.3f}, pitch = {:.3f}, yaw = {:.3f} (final cost {:.6f})",
            current_offsets.x() * 180.0 / M_PI,
            current_offsets.y() * 180.0 / M_PI,
            current_offsets.z() * 180.0 / M_PI,
            cost));

        // --- Diagnostic: did the optimisation find a genuinely better fit than the starting (warm-start) config? ---
        // Re-associate and score each offset set under its OWN associations (not a shared set), then report the RMS
        // re-projection error in metres, normalised for the differing inlier counts. If the optimised RMS is lower,
        // the optimiser is fitting the data better than the warm start; if the warm start (e.g. the manually
        // calibrated values) fits better, the global minimum is not where the optimiser landed - a sign the
        // objective is biased rather than the search being mis-initialised. NOTE: associate() rebuilds the shared
        // `samples` buffer, but the ICP loop is finished so clobbering it here is harmless.
        const auto evaluate = [this](const Eigen::Vector3d& o) {
            associate(o);
            const size_t n   = samples.size();
            const double c   = total_cost(o);
            const double rms = n > 0 ? std::sqrt(c / static_cast<double>(n)) : 0.0;
            return std::make_tuple(n, c, rms);
        };
        const auto [warm_n, warm_cost, warm_rms] = evaluate(warm_start_offsets);
        const auto [opt_n, opt_cost, opt_rms]    = evaluate(current_offsets);
        log<INFO>(
            fmt::format("Cost comparison | warm start (deg) roll = {:.3f}, pitch = {:.3f}, yaw = {:.3f}: "
                        "{} samples, cost = {:.6f}, RMS = {:.4f} m",
                        warm_start_offsets.x() * 180.0 / M_PI,
                        warm_start_offsets.y() * 180.0 / M_PI,
                        warm_start_offsets.z() * 180.0 / M_PI,
                        warm_n,
                        warm_cost,
                        warm_rms));
        log<INFO>(
            fmt::format("Cost comparison | optimised  (deg) roll = {:.3f}, pitch = {:.3f}, yaw = {:.3f}: "
                        "{} samples, cost = {:.6f}, RMS = {:.4f} m",
                        current_offsets.x() * 180.0 / M_PI,
                        current_offsets.y() * 180.0 / M_PI,
                        current_offsets.z() * 180.0 / M_PI,
                        opt_n,
                        opt_cost,
                        opt_rms));

        if (calibrated) {
            write_offsets(current_offsets);
            log<INFO>("New offsets written to config. Calibration complete.");
        }
    }

    double ExtrinsicsCalibration::total_cost(const Eigen::Vector3d& offsets) const {
        double cost = 0.0;
        for (const auto& sample : samples) {
            cost += (project_to_field(offsets, sample) - sample.rLFf).squaredNorm();
        }
        return cost;
    }

    double ExtrinsicsCalibration::objective(const std::vector<double>& x, std::vector<double>& grad, void* data) {
        (void) grad;  // BOBYQA is derivative-free
        auto* self = static_cast<ExtrinsicsCalibration*>(data);
        // Optimisation variables are the offset deltas; evaluate at (current_offsets + delta)
        const Eigen::Vector3d delta(x[0], x[1], x[2]);
        return self->total_cost(self->current_offsets + delta);
    }

    std::tuple<Eigen::Vector3d, double, nlopt::result> ExtrinsicsCalibration::optimise() {
        nlopt::opt opt(nlopt::LN_BOBYQA, n_params);
        opt.set_min_objective(ExtrinsicsCalibration::objective, this);
        opt.set_xtol_rel(cfg.xtol_rel);
        opt.set_ftol_rel(cfg.ftol_rel);
        opt.set_maxeval(static_cast<int>(cfg.maxeval));

        // Bound the offset deltas about zero
        std::vector<double> lb{-cfg.offset_bounds.x(), -cfg.offset_bounds.y(), -cfg.offset_bounds.z()};
        std::vector<double> ub{cfg.offset_bounds.x(), cfg.offset_bounds.y(), cfg.offset_bounds.z()};
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);

        // Initial guess is zero delta (i.e. the current offsets)
        std::vector<double> x{0.0, 0.0, 0.0};
        double final_cost    = 0.0;
        nlopt::result result = nlopt::FAILURE;
        try {
            result = opt.optimize(x, final_cost);
        }
        catch (const std::exception& e) {
            log<ERROR>(fmt::format("Optimisation failed: {}", e.what()));
        }

        const Eigen::Vector3d offsets = current_offsets + Eigen::Vector3d(x[0], x[1], x[2]);
        return {offsets, final_cost, result};
    }

    void ExtrinsicsCalibration::write_offsets(const Eigen::Vector3d& offsets) {
        try {
            YAML::Node cam_cfg = YAML::LoadFile(camera_config_path);
            // Write back as "<degrees> * pi / 180" expression strings so the config stays human-readable
            cam_cfg["roll_offset"]  = fmt::format("{} * pi / 180", offsets.x() * 180.0 / M_PI);
            cam_cfg["pitch_offset"] = fmt::format("{} * pi / 180", offsets.y() * 180.0 / M_PI);
            cam_cfg["yaw_offset"]   = fmt::format("{} * pi / 180", offsets.z() * 180.0 / M_PI);
            std::ofstream file(camera_config_path);
            file << cam_cfg;
            log<INFO>(fmt::format("Wrote optimised offsets to {}", camera_config_path));
        }
        catch (const std::exception& e) {
            log<ERROR>(fmt::format("Failed to write camera config '{}': {}", camera_config_path, e.what()));
        }
    }

}  // namespace module::tools
