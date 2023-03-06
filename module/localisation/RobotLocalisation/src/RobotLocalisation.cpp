#include "RobotLocalisation.hpp"

#include <Eigen/Dense>
#include <fstream>

#include "extension/Configuration.hpp"

namespace module::localisation {

    using extension::Configuration;

    using message::localisation::Field;
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
            cfg.grid_size   = config["grid_size"].as<double>();
            cfg.n_particles = config["n_particles"].as<int>();

            // Odometry scaling factors
            cfg.scale_x     = config["scale_x"].as<double>();
            cfg.scale_y     = config["scale_y"].as<double>();
            cfg.scale_theta = config["scale_theta"].as<double>();

            cfg.save_map = config["save_map"].as<bool>();

            // Initial state, covariance and process noise
            state                        = config["initial_state"].as<Expression>();
            covariance.diagonal()        = Eigen::Vector3d(config["initial_covariance"].as<Expression>());
            cfg.process_noise.diagonal() = Eigen::Vector3d(config["process_noise"].as<Expression>());

            // Initialise the particles with a multivariate normal distribution
            MultivariateNormal<double, 3> multivariate(state, covariance);
            for (int i = 0; i < cfg.n_particles; i++) {
                particles.push_back(Particle(multivariate.sample(), 1.0));
            }
        });

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

            // Fill the surrounding cells close to the field lines with decreasing occupancy values
            fieldline_map.fill_surrounding_cells(0.25 / cfg.grid_size);

            // Save the map to a csv file
            if (cfg.save_map) {
                std::ofstream file("recordings/fieldline_map.csv");
                file << fieldline_map.get_map();
                file.close();
            }

            if (log_level <= NUClear::DEBUG) {
                // For all points in the map with an occupancy value of 1, display them on graph
                Eigen::MatrixXd fieldline_map_data = fieldline_map.get_map();
                for (int i = 0; i < fieldline_map_data.rows(); i++) {
                    for (int j = 0; j < fieldline_map_data.cols(); j++) {
                        if (fieldline_map_data(i, j) == 1) {
                            emit(graph("Field Line Map", i, j));
                        }
                    }
                }
            }

            // Set the time update time to now
            last_time_update_time = NUClear::clock::now();
        });

        on<Trigger<WalkCommand>>().then([this](const WalkCommand& wc) {
            if (!walk_engine_enabled) {
                const auto current_time = NUClear::clock::now();
                last_time_update_time   = current_time;
                walk_engine_enabled     = true;
            }
            walk_command = wc.command;
        });

        on<Trigger<EnableWalkEngineCommand>>().then([this]() {
            walk_engine_enabled     = true;
            const auto current_time = NUClear::clock::now();
            last_time_update_time   = current_time;
        });

        on<Trigger<DisableWalkEngineCommand>>().then([this]() {
            walk_command        = Eigen::Vector3d::Zero();
            walk_engine_enabled = false;
        });

        on<Trigger<StopCommand>>().then([this]() {
            walk_command        = Eigen::Vector3d::Zero();
            walk_engine_enabled = false;
        });

        on<Trigger<ExecuteGetup>>().then([this]() { falling = true; });

        on<Trigger<KillGetup>>().then([this]() { falling = false; });

        on<Trigger<FieldLines>>().then("Particle Filter", [this](const FieldLines& field_lines) {
            if (!falling) {
                // Time update
                time_update();

                // Add noise to the particles
                add_noise();

                // Convert the unit vectors from vision to points on field plane
                std::vector<Eigen::Vector2d> field_point_observations;
                for (auto point : field_lines.points) {
                    Eigen::Isometry3d Hcw = Eigen::Isometry3d(field_lines.Hcw.cast<double>());
                    auto uPCw             = point.cast<double>();
                    // TODO: Transform the measurements into a frame which is on the field plane below the robot
                    // with x pointing forwards and y pointing left of the robot
                    auto rPCw = ray_to_field_plane(uPCw, Hcw);
                    field_point_observations.push_back(rPCw);
                    if (log_level <= NUClear::DEBUG) {
                        auto cell = observation_relative(state, rPCw);
                        emit(graph("Observation points on map [x,y]:", cell.x(), cell.y()));
                    }
                }

                // Calculate the weight of each particle based on the field line observations
                for (int i = 0; i < cfg.n_particles; i++) {
                    particles[i].weight = calculate_weight(particles[i].state, field_point_observations);
                    if (log_level <= NUClear::DEBUG) {
                        auto particle_cell = observation_relative(particles[i].state, Eigen::Vector2d(0.0, 0.0));
                        emit(graph("Particle " + std::to_string(i), particle_cell.x(), particle_cell.y()));
                    }
                }

                // Resample the particles based on the weights
                resample();

                // Compute the state (mean) and covariance of the particles
                state      = compute_mean();
                covariance = compute_covariance();

                if (log_level <= NUClear::DEBUG) {
                    auto state_cell = observation_relative(state, Eigen::Vector2d(0.0, 0.0));
                    emit(graph("State (x,y)", state_cell.x(), state_cell.y()));
                    // Emit the cell 0.5m in front of the robot to show the direction the robot is facing
                    auto direction_cell = observation_relative(state, Eigen::Vector2d(0.5, 0.0));
                    emit(graph("State (theta)", direction_cell.x(), direction_cell.y()));
                }

                // Build and emit the field message
                auto field(std::make_unique<Field>());
                Eigen::Isometry2d position(Eigen::Isometry2d::Identity());
                position.translation() = Eigen::Vector2d(state.x(), state.y());
                position.linear()      = Eigen::Rotation2Dd(state.z()).toRotationMatrix();
                field->position        = position.matrix();
                field->covariance      = covariance;
                emit(field);
            }
        });
    }


    Eigen::Vector2d RobotLocalisation::ray_to_field_plane(Eigen::Vector3d uPCw, Eigen::Isometry3d Hcw) {
        auto Hwc             = Hcw.inverse();
        Eigen::Vector3d rPCw = uPCw * std::abs(Hwc.translation().z() / uPCw.z());
        return rPCw.head(2);
    }

    Eigen::Vector2i RobotLocalisation::observation_relative(const Eigen::Matrix<double, 3, 1> particle,
                                                            const Eigen::Vector2d observation) {
        // Calculate the position of observation relative to the field [m]
        double c       = cos(particle(2));
        double s       = sin(particle(2));
        double x_field = observation(0) * c - observation(1) * s + particle(0);
        double y_field = observation(0) * s + observation(1) * c + particle(1);

        // Get the associated position in the map [x, y]
        int x_map = fieldline_map.get_length() / 2 - std::round(y_field / cfg.grid_size);
        int y_map = fieldline_map.get_width() / 2 + std::round(x_field / cfg.grid_size);

        return Eigen::Vector2i(x_map, y_map);
    }

    double RobotLocalisation::calculate_weight(const Eigen::Matrix<double, 3, 1> particle,
                                               const std::vector<Eigen::Vector2d>& observations) {
        double weight         = 0;
        double n_observations = observations.size();

        for (auto observation : observations) {
            // Get the position of the particle in the map to check if robot is on the field [x, y]
            Eigen::Vector2i particle_position = observation_relative(particle, Eigen::Vector2d(0.0, 0.0));
            double particle_occupancy = fieldline_map.get_occupancy_value(particle_position.x(), particle_position.y());

            // Get the position of the observation in the map for this particle [x, y]
            Eigen::Vector2i map_position = observation_relative(particle, observation);

            // Get the occupancy value of the observation at this position in the map
            double observation_occupancy = fieldline_map.get_occupancy_value(map_position.x(), map_position.y());

            // If the particle_occupancy is -1 then the robot is outside the map, penalise the particle
            if (particle_occupancy == -1) {
                weight = 0;
            }
            // If the observation_occupancy is -1 then the observation is outside the map, penalise the particle.
            // Reduce the weight of the particle by the 1 / number of observations
            else if (observation_occupancy == -1) {
                weight -= (1.0 / n_observations);
            }
            else if (observation_occupancy > 0) {
                // Increase the weight of the particle by the occupancy/n_observations, such that the maximum weight is
                // between [0, 1]
                weight += observation_occupancy / n_observations;
            }
        }

        // Ensure the weight is positive
        weight = std::max(weight, 0.0);

        return weight;
    }

    Eigen::Vector3d RobotLocalisation::compute_mean() {
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (const auto& particle : particles) {
            mean.x() += particle.state.x();
            mean.y() += particle.state.y();
            mean.z() += particle.state.z();
        }
        return mean / cfg.n_particles;
    }

    Eigen::Matrix<double, 3, 3> RobotLocalisation::compute_covariance() {
        Eigen::Matrix<double, 3, 3> cov_matrix = Eigen::Matrix<double, 3, 3>::Zero();
        for (const auto& particle : particles) {
            const Eigen::Matrix<double, 3, 1> deviation = particle.state - state;
            cov_matrix += deviation * deviation.transpose();
        }
        cov_matrix /= (cfg.n_particles - 1);
        return cov_matrix;
    }

    void RobotLocalisation::time_update() {
        // Calculate the time since the last time update
        using namespace std::chrono;
        const auto current_time = NUClear::clock::now();
        const double dt         = duration_cast<duration<double>>(current_time - last_time_update_time).count();
        last_time_update_time   = current_time;

        // Update the particles using the walk command and the time since the last time update, with some tunable
        // scaling
        for (size_t i = 0; i < particles.size(); i++) {
            double delta_x         = walk_command.x() * dt * cfg.scale_x;
            double delta_y         = walk_command.y() * dt * cfg.scale_y;
            double delta_theta     = walk_command.z() * dt * cfg.scale_theta;
            particles[i].state.x() = particles[i].state.x() + delta_x * cos(particles[i].state.z() + delta_theta)
                                     - delta_y * sin(particles[i].state.z() + delta_theta);
            particles[i].state.y() = particles[i].state.y() + delta_y * cos(particles[i].state.z() + delta_theta)
                                     + delta_x * sin(particles[i].state.z() + delta_theta);
            particles[i].state.z() = particles[i].state.z() + delta_theta;
        }
    }

    void RobotLocalisation::resample() {
        std::vector<double> weights(particles.size());
        for (size_t i = 0; i < particles.size(); i++) {
            weights[i] = particles[i].weight;
        }
        double weight_sum = std::accumulate(weights.begin(), weights.end(), 0.0);
        if (weight_sum == 0) {
            log<NUClear::WARN>("All weights are zero, cannot resample");
            // Add some more noise to the particles
            add_noise();
            return;
        }
        // Normalise the weights so that they sum to 1
        for (size_t i = 0; i < weights.size(); i++) {
            weights[i] /= weight_sum;
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

    void RobotLocalisation::add_noise() {
        MultivariateNormal<double, 3> multivariate(Eigen::Vector3d(0.0, 0.0, 0.0), cfg.process_noise);
        for (auto& particle : particles) {
            auto noise = multivariate.sample();
            particle.state += noise;
        }
    }

}  // namespace module::localisation
