/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#include "FieldLocalisation.hpp"

#include <fstream>

#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::localisation {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::localisation::Field;
    using message::localisation::ResetFieldLocalisation;
    using message::vision::FieldLines;

    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::nusight::graph;
    using utility::support::Expression;

    using namespace std::chrono;

    FieldLocalisation::FieldLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("FieldLocalisation.yaml").then([this](const Configuration& config) {
            this->log_level                     = config["log_level"].as<NUClear::LogLevel>();
            cfg.grid_size                       = config["grid_size"].as<double>();
            cfg.save_map                        = config["save_map"].as<bool>();
            cfg.n_particles                     = config["n_particles"].as<int>();
            cfg.initial_state                   = Eigen::Vector3d(config["initial_state"].as<Expression>());
            cfg.initial_covariance.diagonal()   = Eigen::Vector3d(config["initial_covariance"].as<Expression>());
            cfg.process_noise.diagonal()        = Eigen::Vector3d(config["process_noise"].as<Expression>());
            cfg.starting_side                   = config["starting_side"].as<std::string>();
            cfg.start_time_delay                = config["start_time_delay"].as<double>();
            cfg.use_ground_truth_localisation   = config["use_ground_truth_localisation"].as<bool>();
            filter.model.process_noise_diagonal = config["process_noise"].as<Expression>();
            filter.model.n_particles            = config["n_particles"].as<int>();
        });

        on<Startup, Trigger<FieldDescription>>().then("Update Field Line Map", [this](const FieldDescription& fd) {
            // Generate the field line distance map
            fieldline_distance_map = utility::localisation::setup_fieldline_distance_map(fd, cfg.grid_size);
            if (cfg.save_map) {
                std::ofstream file("recordings/fieldline_map.csv");
                file << fieldline_distance_map.get_map();
                file.close();
            }

            // Set the initial state as either left, right, both sides of the field or manually specified inital state
            auto left_side =
                Eigen::Vector3d((fd.dimensions.field_length / 4), (fd.dimensions.field_width / 2), -M_PI_2);
            auto right_side =
                Eigen::Vector3d((fd.dimensions.field_length / 4), (-fd.dimensions.field_width / 2), M_PI_2);
            switch (cfg.starting_side) {
                case StartingSide::LEFT:
                    cfg.initial_hypotheses.emplace_back(std::make_pair(left_side, cfg.initial_covariance));
                    break;
                case StartingSide::RIGHT:
                    cfg.initial_hypotheses.emplace_back(std::make_pair(right_side, cfg.initial_covariance));
                    break;
                case StartingSide::EITHER:
                    cfg.initial_hypotheses.emplace_back(std::make_pair(left_side, cfg.initial_covariance));
                    cfg.initial_hypotheses.emplace_back(std::make_pair(right_side, cfg.initial_covariance));
                    break;
                case StartingSide::CUSTOM:
                    cfg.initial_hypotheses.emplace_back(std::make_pair(cfg.initial_state, cfg.initial_covariance));
                    break;
                default: log<NUClear::ERROR>("Invalid starting_side specified"); break;
            }
            filter.set_state(cfg.initial_hypotheses);

            last_time_update_time = NUClear::clock::now();
            startup_time          = NUClear::clock::now();
        });

        on<Trigger<ResetFieldLocalisation>>().then([this] { filter.set_state(cfg.initial_hypotheses); });

        on<Trigger<FieldLines>, With<Stability>, With<RawSensors>>().then(
            "Particle Filter",
            [this](const FieldLines& field_lines, const Stability& stability, const RawSensors& raw_sensors) {
                log<NUClear::DEBUG>("Running particle filter");
                log<NUClear::DEBUG>("Field lines: ", field_lines.rPWw.size());
                log<NUClear::DEBUG>("Field localisation state: ", filter.get_state().transpose());
                auto time_since_startup =
                    std::chrono::duration_cast<std::chrono::seconds>(NUClear::clock::now() - startup_time).count();
                bool fallen = stability <= Stability::FALLING;

                // Emit field message using ground truth if available
                if (cfg.use_ground_truth_localisation) {
                    auto field(std::make_unique<Field>());
                    field->Hfw = raw_sensors.localisation_ground_truth.Hfw;
                    emit(field);
                    return;
                }

                // Not a valid time to run localisation
                // if (fallen || field_lines.rPWw.size() < cfg.min_observations
                //     || time_since_startup < cfg.start_time_delay) {
                //     return;
                // }

                // Measurement update (using field line observations)
                for (int i = 0; i < cfg.n_particles; i++) {
                    auto weight = calculate_weight(filter.get_particle(i), field_lines.rPWw);
                    filter.set_particle_weight(weight, i);
                }

                // Time update (includes resampling)
                const double dt =
                    duration_cast<duration<double>>(NUClear::clock::now() - last_time_update_time).count();
                log<NUClear::DEBUG>("Time since last update: ", dt);
                last_time_update_time = NUClear::clock::now();
                if (dt <= 0.0) {
                    return;
                }
                filter.time(dt);

                auto field(std::make_unique<Field>());
                field->Hfw = compute_Hfw(filter.get_state());
                if (log_level <= NUClear::DEBUG && raw_sensors.localisation_ground_truth.exists) {
                    debug_field_localisation(field->Hfw, raw_sensors);
                }
                field->covariance = filter.get_covariance();
                if (log_level <= NUClear::DEBUG) {
                    field->particles = filter.get_particles_as_vector();
                }
                emit(field);
            });
    }

    void FieldLocalisation::debug_field_localisation(Eigen::Isometry3d Hfw, const RawSensors& raw_sensors) {
        const Eigen::Isometry3d true_Hfw = Eigen::Isometry3d(raw_sensors.localisation_ground_truth.Hfw);
        // Determine translational distance error
        Eigen::Vector3d true_rFWw  = true_Hfw.translation();
        Eigen::Vector3d rFWw       = (Hfw.translation());
        Eigen::Vector3d error_rFWw = (true_rFWw - rFWw).cwiseAbs();
        // Determine yaw, pitch, and roll error
        Eigen::Vector3d true_Rfw  = mat_to_rpy_intrinsic(true_Hfw.rotation());
        Eigen::Vector3d Rfw       = mat_to_rpy_intrinsic(Hfw.rotation());
        Eigen::Vector3d error_Rfw = (true_Rfw - Rfw).cwiseAbs();
        double quat_rot_error     = Eigen::Quaterniond(true_Hfw.linear() * Hfw.inverse().linear()).w();

        // Graph translation and error from ground truth
        emit(graph("Hfw true translation (rFWw)", true_rFWw.x(), true_rFWw.y(), true_rFWw.z()));
        emit(graph("Hfw translation error", error_rFWw.x(), error_rFWw.y(), error_rFWw.z()));
        // Graph rotation and error from ground truth
        emit(graph("Rfw true angles (rpy)", true_Rfw.x(), true_Rfw.y(), true_Rfw.z()));
        emit(graph("Rfw error (rpy)", error_Rfw.x(), error_Rfw.y(), error_Rfw.z()));
        emit(graph("Quaternion rotational error", quat_rot_error));
    }

    Eigen::Isometry3d FieldLocalisation::compute_Hfw(const Eigen::Vector3d& particle) {
        return Eigen::Translation<double, 3>(particle.x(), particle.y(), 0)
               * Eigen::AngleAxis<double>(particle.z(), Eigen::Matrix<double, 3, 1>::UnitZ());
    }

    Eigen::Vector2i FieldLocalisation::position_in_map(const Eigen::Vector3d& particle, const Eigen::Vector3d& rPWw) {
        // Transform observations from world {w} to field {f} space
        Eigen::Vector3d rPFf = compute_Hfw(particle) * rPWw;

        // Get the associated index in the field line map [x, y]
        // Note: field space is placed in centre of the field, whereas the field line map origin (x,y) is top left
        // corner of discretised field
        return Eigen::Vector2i(fieldline_distance_map.get_length() / 2 - std::round(rPFf(1) / cfg.grid_size),
                               fieldline_distance_map.get_width() / 2 + std::round(rPFf(0) / cfg.grid_size));
    }

    double FieldLocalisation::calculate_weight(const Eigen::Vector3d& particle,
                                               const std::vector<Eigen::Vector3d>& observations) {
        double weight = 0;
        for (auto rORr : observations) {
            // Get the position [x, y] of the observation in the map for this particle
            Eigen::Vector2i map_position = position_in_map(particle, rORr);
            weight += std::pow(fieldline_distance_map.get_occupancy_value(map_position.x(), map_position.y()), 2);
        }
        return 1.0 / (weight + std::numeric_limits<double>::epsilon());
    }
}  // namespace module::localisation
