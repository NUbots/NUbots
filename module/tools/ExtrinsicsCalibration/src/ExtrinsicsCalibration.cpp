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
#include <chrono>
#include <cmath>
#include <fmt/format.h>
#include <fstream>
#include <limits>
#include <nlopt.hpp>
#include <tinyrobotics/kinematics.hpp>
#include <tinyrobotics/parser.hpp>

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"

#include "utility/algorithm/assignment.hpp"
#include "utility/input/FrameID.hpp"
#include "utility/math/euler.hpp"
#include "utility/platform/aliases.hpp"
#include "utility/support/network.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::tools {

    using extension::Configuration;

    using message::input::Sensors;
    using message::support::FieldDescription;
    using message::vision::FieldIntersection;
    using message::vision::FieldIntersections;

    using utility::input::FrameID;
    using utility::localisation::Landmark;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::support::Expression;

    ExtrinsicsCalibration::ExtrinsicsCalibration(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("ExtrinsicsCalibration.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.urdf_path           = config["urdf_path"].as<std::string>();
            cfg.camera              = config["camera"].as<std::string>();
            cfg.is_left_camera      = cfg.camera == "Left";
            cfg.field_yaw           = config["field_yaw"].as<Expression>();
            cfg.start_delay         = config["start_delay"].as<double>();
            cfg.collection_duration = config["collection_duration"].as<double>();
            cfg.min_samples         = config["min_samples"].as<size_t>();

            cfg.max_association_distance = config["max_association_distance"].as<double>();
            cfg.offset_bounds            = Eigen::Vector3d(config["offset_bounds"].as<Expression>());

            cfg.xtol_rel = config["opt"]["xtol_rel"].as<double>();
            cfg.ftol_rel = config["opt"]["ftol_rel"].as<double>();
            cfg.maxeval  = config["opt"]["maxeval"].as<size_t>();

            cfg.write_config = config["write_config"].as<bool>();

            // Compute the offset-free base Hpc (head-pitch {p} from camera {c}) using forward kinematics, the
            // same way the Camera module does before applying the extrinsic offsets.
            auto nugus_model = tinyrobotics::import_urdf<double, 20>(cfg.urdf_path);
            auto camera_frame =
                cfg.is_left_camera ? std::string("left_camera") : std::string("right_camera");
            auto Hpc = tinyrobotics::forward_kinematics<double, 20>(nugus_model,
                                                                    nugus_model.home_configuration(),
                                                                    camera_frame,
                                                                    std::string("head"));
            Hpc_base = Eigen::Isometry3d(Hpc.matrix());

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
            landmarks    = utility::localisation::setup_field_landmarks(fd);
            startup_time = NUClear::clock::now();
            log<INFO>(fmt::format("Loaded {} ground-truth field landmarks", landmarks.size()));
        });

        // Main calibration loop: collect associated detections while the robot is static at the field centre,
        // then run the optimisation once enough data has been gathered.
        on<Trigger<FieldIntersections>, With<Sensors>, With<FieldDescription>, Single>().then(
            "ExtrinsicsCalibration",
            [this](const FieldIntersections& field_intersections, const Sensors& sensors, const FieldDescription&) {
                if (state == State::DONE || landmarks.empty()) {
                    return;
                }

                const auto now     = NUClear::clock::now();
                const double since_startup =
                    std::chrono::duration_cast<std::chrono::duration<double>>(now - startup_time).count();

                // Wait for the configured start delay before collecting
                if (state == State::WAITING) {
                    if (since_startup < cfg.start_delay) {
                        return;
                    }
                    state            = State::COLLECTING;
                    collection_start = now;
                    log<INFO>("Started collecting field landmark detections");
                }

                // --- Build the known transforms for this frame ---
                // Offset-free world {w} from head-pitch {p}, computed the same way as the Camera module
                Eigen::Isometry3d Htw(sensors.Htw);
                Eigen::Isometry3d Htp(sensors.Htx[FrameID::HEAD_PITCH]);
                Eigen::Isometry3d Hwp = Htw.inverse() * Htp;

                // Known field {f} from world {w}, synthesised from the placement assumption: the torso is at the
                // field centre (x = y = 0) facing the goal (field_yaw), with roll/pitch/height taken from sensors.
                Eigen::Isometry3d Hwt = Htw.inverse();
                Eigen::Vector3d rpy   = mat_to_rpy_intrinsic(Hwt.rotation());
                Eigen::Isometry3d Hft = Eigen::Isometry3d::Identity();
                Hft.linear()          = rpy_intrinsic_to_mat(Eigen::Vector3d(rpy.x(), rpy.y(), cfg.field_yaw));
                Hft.translation()     = Eigen::Vector3d(0.0, 0.0, Hwt.translation().z());
                Eigen::Isometry3d Hfw = Hft * Htw;

                // Associate this frame's detections with the known landmarks and store the matched samples
                associate_and_store(field_intersections, Hwp, Hfw);

                // Once the collection window has elapsed, run the optimisation
                const double collecting =
                    std::chrono::duration_cast<std::chrono::duration<double>>(now - collection_start).count();
                if (collecting >= cfg.collection_duration) {
                    state = State::DONE;
                    log<INFO>(fmt::format("Collection complete: {} associated samples", samples.size()));

                    if (samples.size() < cfg.min_samples) {
                        log<ERROR>(fmt::format("Not enough samples to calibrate ({} < {})",
                                               samples.size(),
                                               cfg.min_samples));
                        return;
                    }

                    auto [offsets, cost] = optimise();
                    log<INFO>(fmt::format(
                        "Optimised offsets (deg): roll = {:.3f}, pitch = {:.3f}, yaw = {:.3f} (final cost {:.6f})",
                        offsets.x() * 180.0 / M_PI,
                        offsets.y() * 180.0 / M_PI,
                        offsets.z() * 180.0 / M_PI,
                        cost));

                    if (cfg.write_config) {
                        write_offsets(offsets);
                    }
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

    void ExtrinsicsCalibration::associate_and_store(const FieldIntersections& field_intersections,
                                                    const Eigen::Isometry3d& Hwp,
                                                    const Eigen::Isometry3d& Hfw) {
        if (field_intersections.intersections.empty()) {
            return;
        }

        // Recover each detection's offset-independent camera-frame ray and project it to field space at the
        // current offsets to build the association cost matrix.
        const Eigen::Isometry3d Hcw(field_intersections.Hcw);
        std::vector<Sample> frame_samples;
        frame_samples.reserve(field_intersections.intersections.size());
        for (const auto& intersection : field_intersections.intersections) {
            Sample s;
            s.uICc = (Hcw * intersection.rIWw).normalized();
            s.Hwp  = Hwp;
            s.Hfw  = Hfw;
            frame_samples.push_back(s);
        }

        // Cost matrix (detection index -> landmark index). Same-type pairings only; everything else is set to a
        // large finite cost to avoid overflow.
        constexpr double MAX_ANTIOVERFLOW_COST = 1e9;
        Eigen::MatrixXd cost_matrix(frame_samples.size(), landmarks.size());
        for (size_t d = 0; d < frame_samples.size(); ++d) {
            const Eigen::Vector3d rIFf = project_to_field(current_offsets, frame_samples[d]);
            for (size_t l = 0; l < landmarks.size(); ++l) {
                cost_matrix(d, l) = (landmarks[l].type == field_intersections.intersections[d].type)
                                        ? (landmarks[l].rLFf - rIFf).norm()
                                        : MAX_ANTIOVERFLOW_COST;
            }
        }

        const auto assignment = utility::algorithm::determine_assignment(cost_matrix);
        for (const auto& [detection_index, landmark_index] : assignment) {
            const double cost = cost_matrix(detection_index, landmark_index);
            if (cost < cfg.max_association_distance) {
                Sample s = frame_samples[detection_index];
                s.rLFf   = landmarks[landmark_index].rLFf;
                samples.push_back(s);
            }
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

    std::pair<Eigen::Vector3d, double> ExtrinsicsCalibration::optimise() {
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
        double final_cost = 0.0;
        try {
            opt.optimize(x, final_cost);
        }
        catch (const std::exception& e) {
            log<ERROR>(fmt::format("Optimisation failed: {}", e.what()));
        }

        const Eigen::Vector3d offsets = current_offsets + Eigen::Vector3d(x[0], x[1], x[2]);
        return {offsets, final_cost};
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
