/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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

#include "FieldLocalisationIntersections.hpp"

#include <cmath>
#include <fstream>

#include "extension/Configuration.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::localisation {

    using extension::Configuration;
    using message::behaviour::state::Stability;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::localisation::ResetFieldLocalisation;
    using message::support::FieldDescription;
    using message::vision::FieldIntersections;
    using message::vision::FieldLines;
    using message::vision::Goals;
    using utility::localisation::OccupancyMap;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::nusight::graph;

    FieldLocalisationIntersections::FieldLocalisationIntersections(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), rng(std::random_device{}()) {

        on<Configuration>("FieldLocalisationIntersections.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.num_particles = config["num_particles"].as<int>();
            cfg.min_particles = config["min_particles"].as<int>();
            cfg.grid_size = config["grid_size"].as<double>();
            cfg.save_map = config["save_map"].as<bool>();
            cfg.starting_side = config["starting_side"].as<std::string>();
            cfg.start_time_delay = std::chrono::seconds(config["start_time_delay"].as<int>());
            
            std::vector<double> initial_state = config["initial_state"].as<std::vector<double>>();
            cfg.initial_state = Eigen::Vector3d(initial_state[0], initial_state[1], initial_state[2]);
            
            cfg.min_field_line_points = config["min_field_line_points"].as<int>();
            cfg.random_particle_injection_rate = config["random_particle_injection_rate"].as<double>();
            cfg.process_noise = config["process_noise"].as<std::vector<double>>();
            
            std::vector<double> base_noise = config["base_noise"].as<std::vector<double>>();
            cfg.base_noise = Eigen::Vector3d(base_noise[0], base_noise[1], base_noise[2]);
            
            cfg.field_line_sigma = config["field_line_sigma"].as<double>();
            cfg.field_line_rand_prob = config["field_line_rand_prob"].as<double>();
            cfg.intersection_sigma = config["intersection_sigma"].as<double>();
            cfg.goal_post_sigma = config["goal_post_sigma"].as<double>();
        });

        on<Startup, Trigger<FieldDescription>>().then("Update Field Line Map", [this](const FieldDescription& fd) {
            fieldline_distance_map = utility::localisation::setup_fieldline_distance_map(fd, cfg.grid_size);
            landmarks = utility::localisation::setup_field_landmarks(fd);
            
            if (cfg.save_map) {
                std::ofstream file("recordings/fieldline_map.csv");
                file << fieldline_distance_map.get_map();
                file.close();
            }

            Eigen::Vector3d own_left_goal(fd.dimensions.field_length / 2, fd.dimensions.goal_width / 2, 0);
            Eigen::Vector3d own_right_goal(fd.dimensions.field_length / 2, -fd.dimensions.goal_width / 2, 0);
            Eigen::Vector3d opp_left_goal(-fd.dimensions.field_length / 2, fd.dimensions.goal_width / 2, 0);
            Eigen::Vector3d opp_right_goal(-fd.dimensions.field_length / 2, -fd.dimensions.goal_width / 2, 0);
            own_goal_posts.left = own_left_goal;
            own_goal_posts.right = own_right_goal;
            opp_goal_posts.left = opp_left_goal;
            opp_goal_posts.right = opp_right_goal;

            initialize_particles(fd);
        });

        on<Trigger<ResetFieldLocalisation>, With<FieldDescription>>().then([this](const FieldDescription& fd) {
            log<INFO>("Resetting field localisation");
            initialize_particles(fd);
        });

        on<Trigger<FieldIntersections>,
           With<Stability>,
           With<FieldDescription>,
           With<Sensors>,
           Single>()
            .then("Particle Filter Update", [this](const FieldIntersections& field_intersections,
                                                   const Stability& stability,
                                                   const FieldDescription& fd,
                                                   const Sensors& sensors) {
                if (startup) {
                    startup = false;
                }

                if (stability <= Stability::FALLING || field_intersections.intersections.empty()) {
                    last_Hrw_valid = false;
                    return;
                }

                time_update(sensors.Hrw);
                auto intersections_ptr = std::make_shared<FieldIntersections>(field_intersections);
                measurement_update(intersections_ptr);
                resample(fd);

                // Compute mean state
                double sum_x = 0;
                double sum_y = 0;
                double sum_sin = 0;
                double sum_cos = 0;

                for (const auto& p : particles) {
                    sum_x += p.state.x();
                    sum_y += p.state.y();
                    sum_sin += std::sin(p.state.z());
                    sum_cos += std::cos(p.state.z());
                }

                Eigen::Vector3d mean_state(sum_x / particles.size(),
                                           sum_y / particles.size(),
                                           std::atan2(sum_sin, sum_cos));

                // Compute covariance
                Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
                for (const auto& p : particles) {
                    Eigen::Vector3d diff = p.state - mean_state;
                    diff.z() = utility::math::angle::normalise_angle(diff.z());
                    covariance += diff * diff.transpose();
                }
                covariance /= particles.size();

                auto field = std::make_unique<Field>();
                field->Hfw = compute_Hfw(mean_state);
                field->covariance = covariance;
                field->uncertainty = covariance.trace();
                
                // Add particles for NUsight debugging
                for (const auto& p : particles) {
                    field->particles.push_back(p.state);
                }

                emit(field);
                
                if (log_level <= DEBUG) {
                    emit(graph("MCL Mean State", mean_state.x(), mean_state.y(), mean_state.z()));
                    emit(graph("MCL Uncertainty", covariance.trace()));
                }
            });
    }

    void FieldLocalisationIntersections::initialize_particles(const FieldDescription& fd) {
        particles.clear();
        particles.reserve(cfg.num_particles);
        double weight = 1.0 / cfg.num_particles;

        if (cfg.starting_side == "CUSTOM") {
            for (int i = 0; i < cfg.num_particles; ++i) {
                particles.push_back({cfg.initial_state, weight});
            }
        } else {
            // Distribute across the field uniformly
            std::uniform_real_distribution<double> x_dist(-fd.dimensions.field_length / 2, fd.dimensions.field_length / 2);
            std::uniform_real_distribution<double> y_dist(-fd.dimensions.field_width / 2, fd.dimensions.field_width / 2);
            std::uniform_real_distribution<double> theta_dist(-M_PI, M_PI);

            for (int i = 0; i < cfg.num_particles; ++i) {
                particles.push_back({Eigen::Vector3d(x_dist(rng), y_dist(rng), theta_dist(rng)), weight});
            }
        }
        last_Hrw_valid = false;
        startup = false;
    }

    Eigen::Isometry3d FieldLocalisationIntersections::compute_Hfw(const Eigen::Vector3d& state) const {
        return Eigen::Translation<double, 3>(state.x(), state.y(), 0)
               * Eigen::AngleAxis<double>(state.z(), Eigen::Vector3d::UnitZ());
    }

    void FieldLocalisationIntersections::time_update(const Eigen::Isometry3d& Hrw) {
        if (!last_Hrw_valid) {
            last_Hrw = Hrw;
            last_Hrw_valid = true;
            return;
        }

        // Calculate odometry difference
        Eigen::Isometry3d delta_H = last_Hrw.inverse() * Hrw;
        double dx = delta_H.translation().x();
        double dy = delta_H.translation().y();
        double dtheta = mat_to_rpy_intrinsic(delta_H.rotation()).z();
        double distance = std::sqrt(dx * dx + dy * dy);

        // Standard deviations for noise
        double sigma_x = std::max(cfg.base_noise.x(), distance * cfg.process_noise[0]);
        double sigma_y = std::max(cfg.base_noise.y(), distance * cfg.process_noise[1]);
        double sigma_theta = std::max(cfg.base_noise.z(), distance * cfg.process_noise[2] + std::abs(dtheta) * cfg.process_noise[3]);

        std::normal_distribution<double> noise_x(0, sigma_x);
        std::normal_distribution<double> noise_y(0, sigma_y);
        std::normal_distribution<double> noise_theta(0, sigma_theta);

        for (auto& p : particles) {
            // Apply odometry noise. Since Hfw represents world to field, and world drifted relative to robot,
            // Hfw should drift in the opposite sense or just add random walk.
            // For simplicity, we model the uncertainty as random walk proportional to movement.
            p.state.x() += noise_x(rng);
            p.state.y() += noise_y(rng);
            p.state.z() += noise_theta(rng);
            p.state.z() = utility::math::angle::normalise_angle(p.state.z());
        }

        last_Hrw = Hrw;
    }

    double FieldLocalisationIntersections::compute_particle_weight(const Eigen::Vector3d& state,
                                                                   const std::shared_ptr<const FieldIntersections>& intersections) {
        double log_weight = 0.0;
        Eigen::Isometry3d Hfw = compute_Hfw(state);

        // Intersections
        if (intersections) {
            for (const auto& intersection : intersections->intersections) {
                Eigen::Vector3d rIFf = Hfw * intersection.rIWw;
                
                // Find closest landmark
                double min_dist = std::numeric_limits<double>::max();
                for (const auto& landmark : landmarks) {
                    double dist = (rIFf - landmark.rLFf).norm();
                    if (dist < min_dist) {
                        min_dist = dist;
                    }
                }
                
                double prob = std::exp(-(min_dist * min_dist) / (2 * cfg.intersection_sigma * cfg.intersection_sigma));
                prob = prob * (1.0 - cfg.field_line_rand_prob) + cfg.field_line_rand_prob;
                log_weight += std::log(prob);
            }
        }

        return log_weight;
    }

    void FieldLocalisationIntersections::measurement_update(const std::shared_ptr<const FieldIntersections>& intersections) {
        double max_log_weight = -std::numeric_limits<double>::max();
        
        for (auto& p : particles) {
            p.weight = compute_particle_weight(p.state, intersections);
            if (p.weight > max_log_weight) {
                max_log_weight = p.weight;
            }
        }

        // Log-sum-exp trick and conversion to linear weights
        double sum_weights = 0.0;
        for (auto& p : particles) {
            p.weight = std::exp(p.weight - max_log_weight);
            sum_weights += p.weight;
        }

        // Normalize
        if (sum_weights > 0) {
            for (auto& p : particles) {
                p.weight /= sum_weights;
            }
        } else {
            // If all weights are zero (shouldn't happen with rand_prob), uniform weights
            double w = 1.0 / particles.size();
            for (auto& p : particles) {
                p.weight = w;
            }
        }
    }

    void FieldLocalisationIntersections::resample(const FieldDescription& fd) {
        std::vector<Particle> new_particles;
        new_particles.reserve(cfg.num_particles);

        std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);
        std::uniform_real_distribution<double> x_dist(-fd.dimensions.field_length / 2, fd.dimensions.field_length / 2);
        std::uniform_real_distribution<double> y_dist(-fd.dimensions.field_width / 2, fd.dimensions.field_width / 2);
        std::uniform_real_distribution<double> theta_dist(-M_PI, M_PI);

        // Systematic resampling
        double step = 1.0 / cfg.num_particles;
        double r = uniform_dist(rng) * step;
        double c = particles[0].weight;
        int i = 0;

        for (int m = 0; m < cfg.num_particles; ++m) {
            double u = r + m * step;
            while (u > c && i < static_cast<int>(particles.size()) - 1) {
                i++;
                c += particles[i].weight;
            }

            // Inject random particles for kidnapping recovery
            if (uniform_dist(rng) < cfg.random_particle_injection_rate) {
                new_particles.push_back({Eigen::Vector3d(x_dist(rng), y_dist(rng), theta_dist(rng)), 1.0 / cfg.num_particles});
            } else {
                new_particles.push_back({particles[i].state, 1.0 / cfg.num_particles});
            }
        }

        particles = std::move(new_particles);
    }

}  // namespace module::localisation
