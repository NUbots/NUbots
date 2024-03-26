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
#include "message/input/Buttons.hpp"
#include "message/localisation/Field.hpp"
#include "message/output/Buzzer.hpp"
#include "message/support/FieldDescription.hpp"

namespace module::localisation {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::localisation::Field;
    using message::localisation::ResetFieldLocalisation;
    using message::support::FieldDescription;
    using message::vision::FieldLines;

    using message::input::ButtonLeftDown;
    using message::input::ButtonLeftUp;
    using message::localisation::ResetFieldLocalisation;
    using message::output::Buzzer;

    using utility::math::stats::MultivariateNormal;
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
            filter.model.process_noise_diagonal = config["process_noise"].as<Expression>();
            filter.model.n_particles            = config["n_particles"].as<int>();
            cfg.buzzer.localisation_reset_frequency = config["buzzer"]["localisation_reset_frequency"].as<float>();
        });

        on<Startup, Trigger<FieldDescription>>().then("Update Field Line Map", [this](const FieldDescription& fd) {
            // Generate the field line distance map
            setup_fieldline_distance_map(fd);
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

        // When the left (black) button is pressed, reset localisation and ring the buzzer after
        on<Trigger<ButtonLeftDown>>().then([this]() {
            // Reset localisation and ring the buzzer
            emit(std::make_unique<ResetFieldLocalisation>());
            emit(std::make_unique<Buzzer>(cfg.buzzer.localisation_reset_frequency));
        });

        // Silence the buzzer after the user lets go of the left (black) pin
        on<Trigger<ButtonLeftUp>>().then([this]() { emit(std::make_unique<Buzzer>(0)); });

        on<Trigger<ResetFieldLocalisation>>().then([this] {
            filter.set_state(cfg.initial_hypotheses);
            log<NUClear::WARN>("Field localisation reset completed.");
        });

        on<Trigger<FieldLines>, With<Stability>>().then(
            "Particle Filter",
            [this](const FieldLines& field_lines, const Stability& stability) {
                auto time_since_startup =
                    std::chrono::duration_cast<std::chrono::seconds>(NUClear::clock::now() - startup_time).count();
                bool fallen = stability == Stability::FALLEN || stability == Stability::FALLING;
                if (!fallen && field_lines.rPWw.size() > cfg.min_observations
                    && time_since_startup > cfg.start_time_delay) {

                    // Measurement update (using field line observations)
                    for (int i = 0; i < cfg.n_particles; i++) {
                        auto weight = calculate_weight(filter.get_particle(i), field_lines.rPWw);
                        filter.set_particle_weight(weight, i);
                    }

                    // Time update (includes resampling)
                    const double dt =
                        duration_cast<duration<double>>(NUClear::clock::now() - last_time_update_time).count();
                    last_time_update_time = NUClear::clock::now();
                    filter.time(dt);

                    auto field(std::make_unique<Field>());
                    field->Hfw        = compute_Hfw(filter.get_state());
                    field->covariance = filter.get_covariance();
                    emit(field);
                }
            });
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
