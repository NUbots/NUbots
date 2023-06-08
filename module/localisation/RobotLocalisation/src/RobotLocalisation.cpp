#include "RobotLocalisation.hpp"

#include <Eigen/Dense>
#include <fstream>

#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/support/FieldDescription.hpp"

namespace module::localisation {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::localisation::AddNoiseToParticles;
    using message::localisation::Field;
    using message::localisation::ResetRobotLocalisation;
    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::ExecuteGetup;
    using message::motion::KillGetup;
    using message::motion::StopCommand;
    using message::motion::WalkCommand;
    using message::support::FieldDescription;
    using message::vision::FieldLines;

    using utility::math::stats::MultivariateNormal;
    using utility::nusight::graph;
    using utility::support::Expression;

    RobotLocalisation::RobotLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("RobotLocalisation.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.grid_size   = config["grid_size"].as<float>();
            cfg.n_particles = config["n_particles"].as<int>();
            cfg.save_map    = config["save_map"].as<bool>();

            // Initial state, covariance and process noise
            cfg.initial_covariance.diagonal() = Eigen::Vector3f(config["initial_covariance"].as<Expression>());
            cfg.process_noise.diagonal()      = Eigen::Vector3f(config["process_noise"].as<Expression>());
            cfg.measurement_noise             = config["measurement_noise"].as<float>();
            cfg.max_range                     = config["max_range"].as<float>();
            cfg.min_observations              = config["min_observations"].as<size_t>();
            cfg.outside_map_penalty_factor    = config["outside_map_penalty_factor"].as<float>();
        });

        on<Trigger<ResetRobotLocalisation>>().then([this] {
            // Reset the particles to the initial state
            state      = cfg.initial_state[0];
            covariance = cfg.initial_covariance;
            particles.clear();

            int n_particles_each = cfg.n_particles / cfg.initial_state.size();

            // Loop over initial states and evenly distribute particles
            for (auto& s : cfg.initial_state) {
                MultivariateNormal<float, 3> multivariate(s, covariance);
                for (int i = 0; i < n_particles_each; i++) {
                    particles.push_back(Particle(multivariate.sample(), 1.0));
                }
            }
        });

        on<Trigger<AddNoiseToParticles>>().then([this] { add_noise(); });

        on<Startup, Trigger<FieldDescription>>().then("Update Field Line Map", [this](const FieldDescription& fd) {
            // Resize map to the field dimensions
            int map_width  = (fd.dimensions.field_width + 2 * fd.dimensions.border_strip_min_width) / cfg.grid_size;
            int map_length = (fd.dimensions.field_length + 2 * fd.dimensions.border_strip_min_width) / cfg.grid_size;
            fieldline_map.resize(map_width, map_length);

            // Calculate line width in grid cells
            int line_width = fd.dimensions.line_width / cfg.grid_size;

            // Add outer field lines
            int field_x0     = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
            int field_y0     = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
            int field_width  = (fd.dimensions.field_width) / cfg.grid_size;
            int field_length = (fd.dimensions.field_length) / cfg.grid_size;
            fieldline_map.add_rectangle(field_x0, field_y0, field_length, field_width, line_width);

            // Add left goal area box
            int left_goal_box_x0 = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
            int left_goal_box_y0 =
                (fd.dimensions.border_strip_min_width + (fd.dimensions.field_width - fd.dimensions.goal_area_width) / 2)
                / cfg.grid_size;
            int left_goal_box_width  = (fd.dimensions.goal_area_length) / cfg.grid_size;
            int left_goal_box_length = (fd.dimensions.goal_area_width) / cfg.grid_size;
            fieldline_map.add_rectangle(left_goal_box_x0,
                                        left_goal_box_y0,
                                        left_goal_box_width,
                                        left_goal_box_length,
                                        line_width);

            // Add right goal area box
            int right_goal_box_x0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.field_length - fd.dimensions.goal_area_length)
                / cfg.grid_size;
            int right_goal_box_y0     = left_goal_box_y0;
            int right_goal_box_width  = left_goal_box_width;
            int right_goal_box_length = left_goal_box_length;
            fieldline_map.add_rectangle(right_goal_box_x0,
                                        right_goal_box_y0,
                                        right_goal_box_width,
                                        right_goal_box_length,
                                        line_width);

            // Add left penalty area box
            int left_penalty_box_x0 = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
            int left_penalty_box_y0 = (fd.dimensions.border_strip_min_width
                                       + (fd.dimensions.field_width - fd.dimensions.penalty_area_width) / 2)
                                      / cfg.grid_size;
            int left_penalty_box_width  = (fd.dimensions.penalty_area_width) / cfg.grid_size;
            int left_penalty_box_length = (fd.dimensions.penalty_area_length) / cfg.grid_size;
            fieldline_map.add_rectangle(left_penalty_box_x0,
                                        left_penalty_box_y0,
                                        left_penalty_box_length,
                                        left_penalty_box_width,
                                        line_width);

            // Add right penalty area box
            int right_penalty_box_x0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.field_length - fd.dimensions.penalty_area_length)
                / cfg.grid_size;
            int right_penalty_box_y0     = left_penalty_box_y0;
            int right_penalty_box_width  = left_penalty_box_width;
            int right_penalty_box_length = left_penalty_box_length;
            fieldline_map.add_rectangle(right_penalty_box_x0,
                                        right_penalty_box_y0,
                                        right_penalty_box_length,
                                        right_penalty_box_width,
                                        line_width);

            // Add centre line
            int centre_line_x0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.field_length / 2) / cfg.grid_size;
            int centre_line_y0     = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
            int centre_line_width  = (fd.dimensions.field_width) / cfg.grid_size;
            int centre_line_length = (fd.dimensions.line_width) / cfg.grid_size;
            fieldline_map.add_rectangle(centre_line_x0,
                                        centre_line_y0,
                                        centre_line_length,
                                        centre_line_width,
                                        line_width);

            // Add left penalty cross in centre of penalty area
            int left_penalty_cross_x0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.penalty_mark_distance) / cfg.grid_size;
            int left_penalty_cross_y0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.field_width / 2) / cfg.grid_size;
            int left_penalty_cross_width = 0.1 / cfg.grid_size;
            fieldline_map.add_cross(left_penalty_cross_x0, left_penalty_cross_y0, left_penalty_cross_width, line_width);

            // Add right penalty cross in centre of penalty area
            int right_penalty_cross_x0 = (fd.dimensions.border_strip_min_width + fd.dimensions.field_length
                                          - fd.dimensions.penalty_mark_distance)
                                         / cfg.grid_size;
            int right_penalty_cross_y0    = left_penalty_cross_y0;
            int right_penalty_cross_width = 0.1 / cfg.grid_size;
            fieldline_map.add_cross(right_penalty_cross_x0,
                                    right_penalty_cross_y0,
                                    right_penalty_cross_width,
                                    line_width);

            // Add centre cross in centre of field
            int centre_cross_x0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.field_length / 2) / cfg.grid_size
                + line_width / 2;
            int centre_cross_y0    = left_penalty_cross_y0;
            int centre_cross_width = 0.1 / cfg.grid_size;
            fieldline_map.add_cross(centre_cross_x0, centre_cross_y0, centre_cross_width, line_width);

            // Add centre circle
            int centre_circle_x0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.field_length / 2) / cfg.grid_size;
            int centre_circle_y0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.field_width / 2) / cfg.grid_size;
            int centre_circle_r = (fd.dimensions.center_circle_diameter / 2) / cfg.grid_size;
            fieldline_map.add_circle(centre_circle_x0, centre_circle_y0, centre_circle_r, line_width);

            // Precompute the distance map
            fieldline_map.create_distance_map(cfg.grid_size);

            // Save the map to a csv file
            if (cfg.save_map) {
                std::ofstream file("recordings/fieldline_map.csv");
                file << fieldline_map.get_map();
                file.close();
            }

            if (log_level <= NUClear::DEBUG) {
                // For all points in the map with an occupancy value of 1, display them on graph
                Eigen::MatrixXf fieldline_map_data = fieldline_map.get_map();
                for (int i = 0; i < fieldline_map_data.rows(); i++) {
                    for (int j = 0; j < fieldline_map_data.cols(); j++) {
                        if (fieldline_map_data(i, j) == 0) {
                            emit(graph("Field Line Map", i, j));
                        }
                    }
                }
            }

            // Left side penalty mark
            cfg.initial_state.emplace_back((fd.dimensions.field_length / 2.0),
                                           (fd.dimensions.field_width / 2.0),
                                           -M_PI_2);

            // Right side penalty mark
            cfg.initial_state.emplace_back((fd.dimensions.field_length / 2.0),
                                           (-fd.dimensions.field_width / 2.0),
                                           M_PI_2);


            // Initialise the particles with a multivariate normal distribution
            state      = cfg.initial_state[0];
            covariance = cfg.initial_covariance;
            particles.clear();

            int n_particles_each = cfg.n_particles / cfg.initial_state.size();

            // Loop over initial states and evenly distribute particles
            for (auto& s : cfg.initial_state) {
                MultivariateNormal<float, 3> multivariate(s, covariance);
                for (int i = 0; i < n_particles_each; i++) {
                    particles.push_back(Particle(multivariate.sample(), 1.0));
                }
            }

            // Set the time update time to now
            last_time_update_time = NUClear::clock::now();
        });

        on<Trigger<ExecuteGetup>>().then([this]() { falling = true; });

        on<Trigger<KillGetup>>().then([this]() { falling = false; });

        on<Trigger<Stability>>().then([this](const Stability& stability) {
            if (stability == Stability::FALLEN) {
                falling = true;
            }
            else {
                falling = false;
            }
        });

        on<Trigger<FieldLines>>().then("Particle Filter", [this](const FieldLines& field_lines) {
            Eigen::Isometry3f Hcw = Eigen::Isometry3f(field_lines.Hcw.cast<float>());
            // Add noise to the particles
            add_noise();
            if (!falling && field_lines.points.size() > cfg.min_observations) {

                // Project the field line observations (uPCr) onto the field plane
                std::vector<Eigen::Vector2f> field_point_observations;
                for (auto point : field_lines.points) {
                    auto uPCw = point.cast<float>();
                    auto rPCw = ray_to_field_plane(uPCw, Hcw);
                    field_point_observations.push_back(rPCw);
                    if (log_level <= NUClear::DEBUG) {
                        auto cell = position_in_map(state, rPCw);
                        emit(graph("Observation points on map [x,y]:", cell.x(), cell.y()));
                    }
                }

                // Calculate the weight of each particle based on the observations occupancy values
                for (int i = 0; i < cfg.n_particles; i++) {
                    particles[i].weight = calculate_weight(particles[i].state, field_point_observations);
                    if (log_level <= NUClear::DEBUG) {
                        auto particle_cell = position_in_map(particles[i].state, Eigen::Vector2f(0.0, 0.0));
                        emit(graph("Particle " + std::to_string(i), particle_cell.x(), particle_cell.y()));
                    }
                }

                // Resample the particles based on the weights
                resample();
            }
            // Compute the state (mean) and covariance of the particles
            state      = compute_mean();
            covariance = compute_covariance();
            if (log_level <= NUClear::DEBUG) {
                // Graph where the world frame is positioned in the map
                auto state_cell = position_in_map(state, Eigen::Vector2f(0.0, 0.0));
                emit(graph("World (x,y)", state_cell.x(), state_cell.y()));

                // Graph the cell 0.5m in front of the world frame to visualise the direction of world frame x
                auto direction_cell = position_in_map(state, Eigen::Vector2f(0.5, 0.0));
                emit(graph("World (theta)", direction_cell.x(), direction_cell.y()));

                // Graph where the robot is positioned in the map
                Eigen::Isometry3f Hwc = Hcw.inverse();
                Eigen::Vector2f rCWw  = Eigen::Vector2f(Hwc.translation().x(), Hwc.translation().y());
                auto robot_cell       = position_in_map(state, rCWw);
                emit(graph("Robot (x,y)", robot_cell.x(), robot_cell.y()));

                // Graph the cell 0.5m in front of the robot to visualise the direction it's facing
                Eigen::Vector3f rPCc  = Eigen::Vector3f(0.5, 0.0, 0.0);
                Eigen::Vector3f rPWw  = Hwc * rPCc;
                auto robot_theta_cell = position_in_map(state, rPWw.head(2));
                emit(graph("Robot (theta)", robot_theta_cell.x(), robot_theta_cell.y()));

                emit(graph("Localisation Uncertainty", covariance.trace()));
            }
            // Build and emit the field message
            auto field(std::make_unique<Field>());
            Eigen::Isometry3f Hfw(Eigen::Isometry3f::Identity());
            Hfw.translation()  = Eigen::Vector3f(state.x(), state.y(), 0);
            Hfw.linear()       = Eigen::AngleAxisf(state.z(), Eigen::Vector3f::UnitZ()).toRotationMatrix();
            field->Hfw         = Hfw.matrix();
            field->covariance  = covariance;
            field->uncertainty = covariance.trace();
            emit(field);
        });
    }

    Eigen::Vector2f RobotLocalisation::ray_to_field_plane(Eigen::Vector3f uPCr, Eigen::Isometry3f Hcw) {
        // Project the field line points onto the field plane
        auto Hwc             = Hcw.inverse();
        Eigen::Vector3f rCWw = Eigen::Vector3f(Hwc.translation().x(), Hwc.translation().y(), 0.0);
        Eigen::Vector3f rPCw = uPCr * std::abs(Hwc.translation().z() / uPCr.z()) + rCWw;
        return rPCw.head(2);
    }

    Eigen::Vector2i RobotLocalisation::position_in_map(const Eigen::Matrix<float, 3, 1> particle,
                                                       const Eigen::Vector2f rPRw) {
        // Transform observations from world {w} to field {f} space
        Eigen::Isometry2f Hfw;
        Hfw.translation()    = Eigen::Vector2f(particle(0), particle(1));
        Hfw.linear()         = Eigen::Rotation2Df(particle(2)).toRotationMatrix();
        Eigen::Vector2f rPFf = Hfw * rPRw;

        // Get the associated position/index in the map [x, y]
        int x_map = fieldline_map.get_length() / 2 - std::round(rPFf(1) / cfg.grid_size);
        int y_map = fieldline_map.get_width() / 2 + std::round(rPFf(0) / cfg.grid_size);
        return Eigen::Vector2i(x_map, y_map);
    }

    float RobotLocalisation::calculate_weight(const Eigen::Matrix<float, 3, 1> particle,
                                              const std::vector<Eigen::Vector2f>& observations) {
        float weight = 0;
        for (auto rORr : observations) {
            // Get the position of the observation in the map for this particle [x, y]
            Eigen::Vector2i map_position = position_in_map(particle, rORr);
            // Get the distance to the closest field line point in the map
            float occupancy_value = fieldline_map.get_occupancy_value(map_position.x(), map_position.y());
            // Check if the observation is within the max range and within the field
            if (occupancy_value != -1 && rORr.norm() < cfg.max_range) {
                weight += std::exp(-0.5 * std::pow(occupancy_value / cfg.measurement_noise, 2))
                          / (2 * M_PI * std::pow(cfg.measurement_noise, 2));
            }
            // If the observation is outside the max range, penalise it
            else {
                weight *= cfg.outside_map_penalty_factor;
            }
        }

        return std::max(weight, 0.0f);
    }

    void RobotLocalisation::resample() {
        std::vector<float> weights(particles.size());
        for (size_t i = 0; i < particles.size(); i++) {
            weights[i] = particles[i].weight;
        }
        float weight_sum = std::accumulate(weights.begin(), weights.end(), 0.0);
        if (weight_sum == 0) {
            log<NUClear::DEBUG>("All weights are zero, cannot resample");
            // Add some more noise to the particles
            add_noise();
            return;
        }
        // Normalise the weights so that they sum to 1
        for (auto& weight : weights) {
            weight /= weight_sum;
        }

        std::vector<Particle> resampled_particles(particles.size());
        std::default_random_engine gen;
        std::discrete_distribution<int> distribution(weights.begin(), weights.end());
        for (size_t i = 0; i < particles.size(); i++) {
            int index              = distribution(gen);
            resampled_particles[i] = particles[index];
        }

        particles = resampled_particles;
    }

    Eigen::Vector3f RobotLocalisation::compute_mean() {
        Eigen::Vector3f mean = Eigen::Vector3f::Zero();
        for (const auto& particle : particles) {
            mean += particle.state;
        }
        return mean / cfg.n_particles;
    }

    Eigen::Matrix<float, 3, 3> RobotLocalisation::compute_covariance() {
        Eigen::Matrix<float, 3, 3> cov_matrix = Eigen::Matrix<float, 3, 3>::Zero();
        for (const auto& particle : particles) {
            const Eigen::Matrix<float, 3, 1> deviation = particle.state - state;
            cov_matrix += deviation * deviation.transpose();
        }
        cov_matrix /= (cfg.n_particles - 1);
        return cov_matrix;
    }

    void RobotLocalisation::add_noise() {
        MultivariateNormal<float, 3> multivariate(Eigen::Vector3f(0.0, 0.0, 0.0), cfg.process_noise);

        for (auto& particle : particles) {
            particle.state += multivariate.sample();
        }
    }

}  // namespace module::localisation
