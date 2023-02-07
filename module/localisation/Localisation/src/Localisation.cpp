#include "Localisation.hpp"

#include <Eigen/Dense>
#include <fstream>

#include "extension/Configuration.hpp"

namespace module::localisation {

    using extension::Configuration;

    using VisionGoal  = message::vision::Goal;
    using VisionGoals = message::vision::Goals;
    using message::support::FieldDescription;
    using VisionLines = message::vision::FieldLines;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::ExecuteGetup;
    using message::motion::KillGetup;
    using message::motion::StopCommand;
    using message::motion::WalkCommand;

    using utility::input::ServoID;
    using utility::math::coordinates::cartesianToPolar;
    using utility::math::coordinates::reciprocalSphericalToCartesian;
    using utility::math::stats::MultivariateNormal;
    using utility::nusight::graph;
    using utility::support::Expression;


    Localisation::Localisation(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Localisation.yaml").then([this](const Configuration& config) {
            this->log_level   = config["log_level"].as<NUClear::LogLevel>();
            cfg.grid_size     = config["grid_size"].as<double>();
            cfg.num_particles = config["num_particles"].as<int>();
            cfg.process_noise = config["process_noise"].as<Expression>();

            // Initial state and covariance
            state                               = config["initial_state"].as<Expression>();
            Eigen::Vector3d covariance_diagonal = config["initial_covariance"].as<Expression>();
            covariance.diagonal() << covariance_diagonal;

            // Initialise the particles
            MultivariateNormal<double, 3> multivariate(state, covariance);
            for (int i = 0; i < cfg.num_particles; i++) {
                particles.push_back(Particle(multivariate.sample(), 1.0 / cfg.num_particles));
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

            // TODO: Add centre cross in centre of field

            // TODO: Add left penalty cross in centre of penalty area

            // TODO: Add right penalty cross in centre of penalty area

            // Add centre circle
            int centre_circle_x0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.field_length / 2) / cfg.grid_size;
            int centre_circle_y0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.field_width / 2) / cfg.grid_size;
            int centre_circle_r = (fd.dimensions.center_circle_diameter / 2) / cfg.grid_size;
            fieldline_map.add_circle(centre_circle_x0, centre_circle_y0, centre_circle_r, line_width);

            // Fill the surrounding cells close to the field lines
            fieldline_map.fill_surrounding_cells(0.2 / cfg.grid_size);

            // --------------------- TEMPORARY: REMOVE LATER ---------------------
            // Open a file in write mode
            std::ofstream file("config/matrix.csv");
            // Write the matrix to the file
            file << fieldline_map.map;
            // Close the file
            file.close();

            // For all points in the map with an occupancy value of 1, display them on graph
            for (int i = 0; i < fieldline_map.map.rows(); i++) {
                for (int j = 0; j < fieldline_map.map.cols(); j++) {
                    if (fieldline_map.map(i, j) == 1) {
                        emit(graph("Fieldline Map", i, j));
                    }
                }
            }

            // Set the update times to now
            last_time_update_time        = NUClear::clock::now();
            last_measurement_update_time = NUClear::clock::now();
        });

        on<Trigger<WalkCommand>>().then([this](const WalkCommand& wc) {
            walk_engine_enabled = true;
            walk_command        = wc.command;
            log<NUClear::INFO>("Walk command received: ",
                               walk_command.x(),
                               ", ",
                               walk_command.y(),
                               ", ",
                               walk_command.z());
        });

        on<Trigger<EnableWalkEngineCommand>>().then([this](const EnableWalkEngineCommand& command) {
            walk_engine_enabled     = true;
            const auto current_time = NUClear::clock::now();
            last_time_update_time   = current_time;
        });

        on<Trigger<DisableWalkEngineCommand>>().then([this](const DisableWalkEngineCommand& command) {
            walk_command        = Eigen::Vector3d::Zero();
            walk_engine_enabled = false;
        });

        on<Trigger<StopCommand>>().then([this](const StopCommand& command) {
            walk_command        = Eigen::Vector3d::Zero();
            walk_engine_enabled = false;
            log<NUClear::INFO>("Stop command received");
        });

        on<Trigger<ExecuteGetup>>().then([this]() { falling = true; });

        on<Trigger<KillGetup>>().then([this]() { falling = false; });

        on<Trigger<VisionLines>, With<FieldDescription>, With<Sensors>>().then(
            "Vision Lines",
            [this](const VisionLines& line_points, const FieldDescription& fd, const Sensors& sensors) {
                /* Perform measurement correction */
                // Identify the line points on the field
                Eigen::Vector3d test_state(0.0, 0.0, 0.0);
                std::vector<Eigen::Vector2d> field_point_observations;
                for (auto point : line_points.points) {
                    Eigen::Isometry3d Hcw = Eigen::Isometry3d(line_points.Hcw.cast<double>());
                    auto uPCw             = point.cast<double>();
                    auto rPCc             = ray2field(uPCw, Hcw);
                    field_point_observations.push_back(rPCc);
                    if (log_level <= NUClear::DEBUG) {
                        auto cell = observation_relative(test_state, rPCc);
                        emit(graph("Observation points [m]:", rPCc.x(), rPCc.y()));
                        emit(graph("Observation points on map [x,y]:", cell.x(), cell.y()));
                    }
                }

                for (int i = 0; i < cfg.num_particles; i++) {
                    // Calculate the weight the particle
                    particles[i].weight = calculate_weight(particles[i].state, field_point_observations);
                    if (log_level <= NUClear::DEBUG) {
                        auto particle_cell = observation_relative(particles[i].state, Eigen::Vector2d(0.0, 0.0));
                        emit(graph("Particle " + std::to_string(i), particle_cell.x(), particle_cell.y()));
                    }
                }

                // Perform time update on all the particles
                if (walk_engine_enabled && !falling) {
                    time_update();
                }

                // Add noise to the particles
                add_noise();

                // Resample
                resample(particles);

                // Calculate the mean state
                Eigen::Vector3d state_mean = Eigen::Vector3d::Zero();
                for (int i = 0; i < cfg.num_particles; i++) {
                    state_mean.x() += particles[i].state.x();
                    state_mean.y() += particles[i].state.y();
                    state_mean.z() += particles[i].state.z();
                }
                state_mean /= cfg.num_particles;
                state = state_mean;

                // Build and emit the field message
                auto field(std::make_unique<Field>());
                Eigen::Isometry2d position(Eigen::Isometry2d::Identity());
                position.translation() = Eigen::Vector2d(state.x(), state.y());
                position.linear()      = Eigen::Rotation2Dd(state.z()).toRotationMatrix();
                field->position        = position.matrix();

                emit(graph("State", state.x(), state.y(), state.z()));
                emit(field);
            });
    }

    /// @brief Converts a unit vector of point from the camera in world space to a (x,y) point relative to the robot on
    /// the field plane
    /// @param uPCw unit vector from the camera to the field point in world space
    /// @param Hcw the camera to world transform
    /// @return the field point measurement (x,y) relative to the robot
    Eigen::Vector2d Localisation::ray2field(Eigen::Vector3d uPCw, Eigen::Isometry3d Hcw) {
        auto Hwc             = Hcw.inverse();
        Eigen::Vector3d rPCw = uPCw * std::abs(Hwc.translation().z() / uPCw.z());
        return rPCw.head(2);
    }


    double Localisation::get_occupancy(const Eigen::Vector2i observation) {
        if (observation.x() < 0 || observation.x() >= fieldline_map.map.cols() || observation.y() < 0
            || observation.y() >= fieldline_map.map.rows()) {
            return 0;
        }
        else {
            return fieldline_map.map(observation.x(), observation.y());
        }
    }

    Eigen::Vector2i Localisation::observation_relative(const Eigen::Matrix<double, 3, 1> particle,
                                                       const Eigen::Vector2d observation) {
        double c = cos(particle(2));
        double s = sin(particle(2));

        // Calculate the position of observation relative to the field [m]
        double x_field = observation(0) * c - observation(1) * s + particle(0);
        double y_field = observation(0) * s + observation(1) * c + particle(1);

        // Get the associated position in the map
        int x_map = fieldline_map.map.rows() / 2 - std::round(y_field / cfg.grid_size);
        int y_map = fieldline_map.map.cols() / 2 + std::round(x_field / cfg.grid_size);

        return Eigen::Vector2i(x_map, y_map);
    }

    /// @brief Get the weight of a particle given a set of observations
    double Localisation::calculate_weight(const Eigen::Matrix<double, 3, 1> particle,
                                          const std::vector<Eigen::Vector2d>& observations) {
        double weight = 0;
        for (auto observation : observations) {
            // Get the position of the observation in the map for this particle
            Eigen::Vector2i map_position = observation_relative(particle, observation);

            // Get the occupancy of the observation
            double occupancy = get_occupancy(map_position);

            // Add the occupancy to the weight of the particle
            weight += occupancy;
        }
        return weight;
    }

    void Localisation::time_update() {

        using namespace std::chrono;
        const auto current_time = NUClear::clock::now();
        const double dt         = duration_cast<duration<double>>(current_time - last_time_update_time).count();
        log<NUClear::DEBUG>("Time since last time update: ", dt);
        last_time_update_time = current_time;

        for (auto particle : particles) {
            // Update the state of the particle
            double delta_x     = walk_command.x() * dt;
            double delta_y     = walk_command.y() * dt;
            double delta_theta = walk_command.z() * dt;

            particle.state.x() = particle.state.x() + delta_x * cos(particle.state.z() + delta_theta)
                                 - delta_y * sin(particle.state.z() + delta_theta);
            particle.state.y() = particle.state.y() + delta_y * cos(particle.state.z() + delta_theta)
                                 + delta_x * sin(particle.state.z() + delta_theta);
            particle.state.z() = particle.state.z() + delta_theta;
        }
    }

    void Localisation::resample(std::vector<Particle>& particles) {
        std::vector<double> weights(particles.size());
        for (int i = 0; i < particles.size(); i++) {
            weights[i] = particles[i].weight;
        }
        double weight_sum = std::accumulate(weights.begin(), weights.end(), 0.0);
        if (weight_sum == 0) {
            std::cout << "All weights are zero, cannot resample" << std::endl;
            return;
        }
        for (int i = 0; i < weights.size(); i++) {
            weights[i] /= weight_sum;
        }
        std::vector<Particle> resampled_particles(particles.size());
        std::random_device rd;
        std::mt19937 generator(rd());
        std::discrete_distribution<> distribution(weights.begin(), weights.end());
        for (int i = 0; i < particles.size(); i++) {
            int index              = distribution(generator);
            resampled_particles[i] = particles[index];
        }

        log<NUClear::DEBUG>("Unsampled particles");
        log_particles(particles, 10);
        log<NUClear::DEBUG>("Resampled particles");
        log_particles(resampled_particles, 10);

        particles = resampled_particles;
    }

    void Localisation::add_noise() {
        Eigen::Matrix<double, 3, 3> process_noise;
        process_noise.diagonal() << cfg.process_noise;
        MultivariateNormal<double, 3> multivariate(Eigen::Vector3d(0.0, 0.0, 0.0), process_noise);
        for (auto& particle : particles) {
            particle.state += multivariate.sample();
        }
    }

    void Localisation::log_particles(const std::vector<Particle>& particles, int number) {
        if (number == -1) {
            number = particles.size();
        }
        for (int i = 0; i < number; i++) {
            log<NUClear::DEBUG>("Particle ",
                                i,
                                "with state: ",
                                particles[i].state.transpose(),
                                " weight: ",
                                particles[i].weight);
        }
    }
}  // namespace module::localisation
