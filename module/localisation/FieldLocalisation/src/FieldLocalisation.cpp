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

namespace module::localisation {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::localisation::Field;
    using message::localisation::ResetFieldLocalisation;
    using message::vision::FieldLines;

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
            cfg.measurement_noise.diagonal()    = Eigen::Vector2d(config["measurement_noise"].as<Expression>());
            cfg.starting_side                   = config["starting_side"].as<std::string>();
            cfg.start_time_delay                = config["start_time_delay"].as<double>();
            filter.model.process_noise_diagonal = config["process_noise"].as<Expression>();
            filter.model.n_particles            = config["n_particles"].as<int>();
            cfg.min_association_distance        = config["min_association_distance"].as<double>();
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

            // ******** Landmarks ********
            setup_field_landmarks(fd);
            for (auto& landmark : landmarks) {
                if (landmark.type == message::vision::FieldIntersection::IntersectionType::L_INTERSECTION) {
                    emit(graph("L Landmark", landmark.position.x(), landmark.position.y()));
                }
                else if (landmark.type == message::vision::FieldIntersection::IntersectionType::T_INTERSECTION) {
                    emit(graph("T Landmark", landmark.position.x(), landmark.position.y()));
                }
                else if (landmark.type == message::vision::FieldIntersection::IntersectionType::X_INTERSECTION) {
                    emit(graph("X Landmark", landmark.position.x(), landmark.position.y()));
                }
            }

            last_time_update_time = NUClear::clock::now();
            startup_time          = NUClear::clock::now();
        });

        on<Trigger<ResetFieldLocalisation>>().then([this] { filter.set_state(cfg.initial_hypotheses); });

        on<Trigger<FieldLines>, With<Stability>>().then(
            "Particle Filter FieldLines",
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
                    if (log_level <= NUClear::DEBUG) {
                        field->particles = filter.get_particles_as_vector();
                    }
                    emit(field);
                }
            });

        on<Trigger<FieldIntersections>, With<Stability>>().then(
            "Particle Filter FieldIntersections",
            [this](const FieldIntersections& field_intersections, const Stability& stability) {
                auto time_since_startup =
                    std::chrono::duration_cast<std::chrono::seconds>(NUClear::clock::now() - startup_time).count();
                bool fallen = stability == Stability::FALLEN || stability == Stability::FALLING;
                if (!fallen && time_since_startup > cfg.start_time_delay) {


                    // Data association (find the closest field intersection of same type in list of particles)
                    for (auto& intersection : field_intersections.intersections) {
                        auto associated_rLFf = find_closest_field_intersection(intersection);
                        if (associated_rLFf) {
                            Eigen::Vector2d rIWw = intersection.rIWw.head<2>();
                            auto rLFf            = associated_rLFf.value();
                            auto state           = filter.get_state();
                            auto Hfw             = Eigen::Translation<double, 2>(state.x(), state.y())
                                       * Eigen::Rotation2D<double>(state.z());
                            auto associated_rLWw = Hfw.inverse() * rLFf;

                            auto rIFf = Hfw * intersection.rIWw.head<2>();
                            emit(graph("Observation", rIFf.x(), rIFf.y()));

                            // log<NUClear::DEBUG>("Associated observation rIWw: ",
                            //                     rIWw.transpose().head<2>(),
                            //                     " with landmark rLWw: ",
                            //                     associated_rLWw.transpose());

                            filter.measure(rIWw, cfg.measurement_noise, associated_rLFf.value());
                        }
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

    void FieldLocalisation::setup_field_landmarks(const FieldDescription& fd) {
        // Half dimensions for easier calculation
        double half_length = fd.dimensions.field_length / 2;
        double half_width  = fd.dimensions.field_width / 2;

        // Corners of the field
        landmarks.push_back({Eigen::Vector2d(-half_length, half_width),
                             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back({Eigen::Vector2d(-half_length, -half_width),
                             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back({Eigen::Vector2d(half_length, half_width),
                             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back({Eigen::Vector2d(half_length, -half_width),
                             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});

        // Mid-points of each sideline
        landmarks.push_back(
            {Eigen::Vector2d(0, half_width), message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});
        landmarks.push_back(
            {Eigen::Vector2d(0, -half_width), message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});

        // X intersection at the center
        landmarks.push_back(
            {Eigen::Vector2d(0, 0), message::vision::FieldIntersection::IntersectionType::X_INTERSECTION});
        landmarks.push_back({Eigen::Vector2d(0, fd.dimensions.center_circle_diameter / 2),
                             message::vision::FieldIntersection::IntersectionType::X_INTERSECTION});
        landmarks.push_back({Eigen::Vector2d(0, -fd.dimensions.center_circle_diameter / 2),
                             message::vision::FieldIntersection::IntersectionType::X_INTERSECTION});

        if (fd.dimensions.penalty_area_length != 0 && fd.dimensions.penalty_area_width != 0) {
            // T intersections from the penalty areas
            landmarks.push_back({Eigen::Vector2d(half_length, fd.dimensions.penalty_area_width / 2),
                                 message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});
            landmarks.push_back({Eigen::Vector2d(half_length, -fd.dimensions.penalty_area_width / 2),
                                 message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});
            landmarks.push_back({Eigen::Vector2d(-half_length, fd.dimensions.penalty_area_width / 2),
                                 message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});
            landmarks.push_back({Eigen::Vector2d(-half_length, -fd.dimensions.penalty_area_width / 2),
                                 message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});
        }

        // T intersections from the goal areas
        landmarks.push_back({Eigen::Vector2d(half_length, fd.dimensions.goal_area_width / 2),
                             message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});
        landmarks.push_back({Eigen::Vector2d(half_length, -fd.dimensions.goal_area_width / 2),
                             message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});
        landmarks.push_back({Eigen::Vector2d(-half_length, fd.dimensions.goal_area_width / 2),
                             message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});
        landmarks.push_back({Eigen::Vector2d(-half_length, -fd.dimensions.goal_area_width / 2),
                             message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});

        // L intersections from the penalty areas
        landmarks.push_back(
            {Eigen::Vector2d(half_length - fd.dimensions.penalty_area_length, fd.dimensions.penalty_area_width / 2),
             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back(
            {Eigen::Vector2d(half_length - fd.dimensions.penalty_area_length, -fd.dimensions.penalty_area_width / 2),
             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back(
            {Eigen::Vector2d(-half_length + fd.dimensions.penalty_area_length, fd.dimensions.penalty_area_width / 2),
             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back(
            {Eigen::Vector2d(-half_length + fd.dimensions.penalty_area_length, -fd.dimensions.penalty_area_width / 2),
             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});

        // L intersections from the goal areas
        landmarks.push_back(
            {Eigen::Vector2d(half_length - fd.dimensions.goal_area_length, fd.dimensions.goal_area_width / 2),
             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back(
            {Eigen::Vector2d(half_length - fd.dimensions.goal_area_length, -fd.dimensions.goal_area_width / 2),
             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back(
            {Eigen::Vector2d(-half_length + fd.dimensions.goal_area_length, fd.dimensions.goal_area_width / 2),
             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back(
            {Eigen::Vector2d(-half_length + fd.dimensions.goal_area_length, -fd.dimensions.goal_area_width / 2),
             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});

        // X intersections from penalty spots
        landmarks.push_back({Eigen::Vector2d(half_length - fd.dimensions.penalty_mark_distance, 0),
                             message::vision::FieldIntersection::IntersectionType::X_INTERSECTION});
        landmarks.push_back({Eigen::Vector2d(-half_length + fd.dimensions.penalty_mark_distance, 0),
                             message::vision::FieldIntersection::IntersectionType::X_INTERSECTION});
        landmarks.push_back(
            {Eigen::Vector2d(0, 0), message::vision::FieldIntersection::IntersectionType::X_INTERSECTION});
    }

    std::optional<Eigen::Vector2d> FieldLocalisation::find_closest_field_intersection(
        const FieldIntersection& observed_intersection) {
        double min_distance = std::numeric_limits<double>::max();
        std::optional<Eigen::Vector2d> closest_landmark_position;

        for (const auto& landmark : landmarks) {
            // Check if the landmark is of the same type as the observed intersection
            if (landmark.type == observed_intersection.type) {
                // Calculate Euclidean distance between the observed intersection and the landmark

                // TODO: This should be done for each particle in the filter, not just the mean
                auto state = filter.get_state();
                auto Hfw   = Eigen::Translation<double, 2>(state.x(), state.y()) * Eigen::Rotation2D<double>(state.z());
                double distance = (landmark.position - Hfw * observed_intersection.rIWw.head<2>())
                                      .norm();  // Adjust for your actual observed intersection position access method

                // If this landmark is closer than the previous closest, update min_distance and
                // closest_landmark_position
                if (distance < min_distance) {
                    min_distance              = distance;
                    closest_landmark_position = landmark.position;
                }
            }
        }

        if (min_distance > cfg.min_association_distance || !closest_landmark_position.has_value()) {
            log<NUClear::DEBUG>("No close landmark found for intersection", observed_intersection.type);
            log<NUClear::DEBUG>("Distance to closest landmark", min_distance);
            return std::nullopt;
        }

        return closest_landmark_position;
    }


}  // namespace module::localisation
