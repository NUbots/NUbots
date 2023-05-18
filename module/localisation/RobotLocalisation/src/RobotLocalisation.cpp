#include "RobotLocalisation.hpp"

#include <Eigen/Dense>
#include <fstream>

#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/support/FieldDescription.hpp"

namespace module::localisation {

    using extension::Configuration;

    using message::behaviour::state::Stability;
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
            cfg.grid_size   = config["grid_size"].as<double>();
            cfg.save_map    = config["save_map"].as<bool>();

            // Initialise the particle filter
            cfg.n_particles                     = config["n_particles"].as<int>();
            cfg.initial_covariance.diagonal()   = Eigen::Vector3d(config["initial_covariance"].as<Expression>());
            cfg.process_noise.diagonal()        = Eigen::Vector3d(config["process_noise"].as<Expression>());
            cfg.measurement_noise               = config["measurement_noise"].as<double>();
            cfg.max_range                       = config["max_range"].as<double>();
            filter.model.process_noise_diagonal = config["process_noise"].as<Expression>();
            filter.model.n_rogues               = config["n_rogues"].as<int>();
            filter.model.reset_range            = config["reset_range"].as<Expression>();
            filter.model.n_particles            = config["n_particles"].as<int>();
        });

        on<Trigger<ResetRobotLocalisation>>().then([this] {
            std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> hypotheses;
            for (const auto& state : cfg.initial_state) {
                hypotheses.emplace_back(std::make_pair(state, cfg.initial_covariance));
            }
            filter.set_state(hypotheses);
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
            // fieldline_map.fill_surrounding_cells(0.25 / cfg.grid_size);

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
                Eigen::MatrixXd fieldline_map_data = fieldline_map.get_map();
                for (int i = 0; i < fieldline_map_data.rows(); i++) {
                    for (int j = 0; j < fieldline_map_data.cols(); j++) {
                        if (fieldline_map_data(i, j) == 0) {
                            emit(graph("Field Line Map", i, j));
                        }
                    }
                }
            }

            // Left side penalty mark
            cfg.initial_state.emplace_back((fd.dimensions.field_length / 2.0) - fd.dimensions.penalty_mark_distance,
                                           (fd.dimensions.field_width / 2.0),
                                           -M_PI_2);

            // Right side penalty mark
            cfg.initial_state.emplace_back((fd.dimensions.field_length / 2.0) - fd.dimensions.penalty_mark_distance,
                                           (-fd.dimensions.field_width / 2.0),
                                           M_PI_2);

            std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> hypotheses;
            for (const auto& state : cfg.initial_state) {
                hypotheses.emplace_back(std::make_pair(state, cfg.initial_covariance));
            }

            // Initialise the particles with a multivariate normal distribution
            filter.set_state(hypotheses);

            // Set the time update time to now
            last_time_update_time = NUClear::clock::now();
        });

        on<Trigger<FieldLines>, With<Stability>>().then(
            "Particle Filter",
            [this](const FieldLines& field_lines, const Stability& stability) {
                if (stability != Stability::FALLEN) {
                    // ********************* Time update *********************

                    // Compute the time since the last time update
                    using namespace std::chrono;
                    const auto curr_time  = NUClear::clock::now();
                    const double seconds  = duration_cast<duration<double>>(curr_time - last_time_update_time).count();
                    last_time_update_time = curr_time;

                    // Perform the time update
                    filter.time(seconds);

                    // ********************* Measurement update *********************

                    // Project the field line observations (uPCr) onto the field plane
                    std::vector<Eigen::Vector2d> field_point_observations;
                    Eigen::Isometry3d Hcw = Eigen::Isometry3d(field_lines.Hcw.cast<double>());
                    for (auto point : field_lines.points) {
                        auto uPCw = point.cast<double>();
                        auto rPCw = ray_to_field_plane(uPCw, Hcw);
                        field_point_observations.push_back(rPCw);
                        if (log_level <= NUClear::DEBUG) {
                            auto cell = position_in_map(filter.get_state(), rPCw);
                            emit(graph("Observation points on map [x,y]:", cell.x(), cell.y()));
                        }
                    }

                    // Calculate the weight of each particle based on the observations occupancy values
                    for (int i = 0; i < cfg.n_particles; i++) {
                        auto particle = filter.get_particle(i);
                        auto weight   = calculate_weight(particle, field_point_observations);
                        filter.set_particle_weight(weight, i);
                        if (log_level <= NUClear::DEBUG) {
                            auto particle_cell = position_in_map(particle, Eigen::Vector2d(0.0, 0.0));
                            emit(graph("Particle " + std::to_string(i), particle_cell.x(), particle_cell.y()));
                        }
                    }

                    // Resample the particles based on the weights
                    filter.resample();

                    // ********************* Debug *********************

                    if (log_level <= NUClear::DEBUG) {
                        // Graph where the world frame is positioned in the map
                        auto state_cell = position_in_map(filter.get_state(), Eigen::Vector2d(0.0, 0.0));
                        emit(graph("World (x,y)", state_cell.x(), state_cell.y()));

                        // Graph the cell 0.5m in front of the world frame to visualise the direction of world frame x
                        auto direction_cell = position_in_map(filter.get_state(), Eigen::Vector2d(0.5, 0.0));
                        emit(graph("World (theta)", direction_cell.x(), direction_cell.y()));

                        // Graph where the robot is positioned in the map
                        Eigen::Isometry3d Hwc = Hcw.inverse();
                        Eigen::Vector2d rCWw  = Eigen::Vector2d(Hwc.translation().x(), Hwc.translation().y());
                        auto robot_cell       = position_in_map(filter.get_state(), rCWw);
                        emit(graph("Robot (x,y)", robot_cell.x(), robot_cell.y()));

                        // Graph the cell 0.5m in front of the robot to visualise the direction it's facing
                        Eigen::Vector3d rPCc  = Eigen::Vector3d(0.5, 0.0, 0.0);
                        Eigen::Vector3d rPWw  = Hwc * rPCc;
                        auto robot_theta_cell = position_in_map(filter.get_state(), rPWw.head(2));
                        emit(graph("Robot (theta)", robot_theta_cell.x(), robot_theta_cell.y()));
                    }

                    // ********************* Emit field message *********************

                    auto field(std::make_unique<Field>());
                    auto state = filter.get_state();
                    Eigen::Isometry3d Hfw(Eigen::Isometry3d::Identity());
                    Hfw.translation() = Eigen::Vector3d(state.x(), state.y(), 0);
                    Hfw.linear()      = Eigen::AngleAxisd(state.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
                    field->Hfw        = Hfw.matrix();
                    field->covariance = filter.get_covariance();
                    emit(field);
                }
            });
    }


    Eigen::Vector2d RobotLocalisation::ray_to_field_plane(Eigen::Vector3d uPCr, Eigen::Isometry3d Hcw) {
        // Project the field line points onto the field plane
        auto Hwc             = Hcw.inverse();
        Eigen::Vector3d rCWw = Eigen::Vector3d(Hwc.translation().x(), Hwc.translation().y(), 0.0);
        Eigen::Vector3d rPCw = uPCr * std::abs(Hwc.translation().z() / uPCr.z()) + rCWw;
        return rPCw.head(2);
    }

    Eigen::Vector2i RobotLocalisation::position_in_map(const Eigen::Matrix<double, 3, 1> particle,
                                                       const Eigen::Vector2d rPRw) {
        // Transform observations from world {w} to field {f} space
        Eigen::Isometry2d Hfw;
        Hfw.translation()    = Eigen::Vector2d(particle(0), particle(1));
        Hfw.linear()         = Eigen::Rotation2Dd(particle(2)).toRotationMatrix();
        Eigen::Vector2d rPFf = Hfw * rPRw;

        // Get the associated position/index in the map [x, y]
        int x_map = fieldline_map.get_length() / 2 - std::round(rPFf(1) / cfg.grid_size);
        int y_map = fieldline_map.get_width() / 2 + std::round(rPFf(0) / cfg.grid_size);
        return Eigen::Vector2i(x_map, y_map);
    }

    double RobotLocalisation::calculate_weight(const Eigen::Matrix<double, 3, 1> particle,
                                               const std::vector<Eigen::Vector2d>& observations) {
        double weight = 0;
        for (auto rORr : observations) {
            // Get the position of the observation in the map for this
            // particle [x, y]
            Eigen::Vector2i map_position = position_in_map(particle, rORr);
            // Get the distance to the closest field line point in the map
            double occupancy_value = fieldline_map.get_occupancy_value(map_position.x(), map_position.y());
            // Check if the observation is within the max range and within
            // the field
            if (occupancy_value != -1 && rORr.norm() < cfg.max_range) {
                double distance_error_norm = std::pow(occupancy_value, 2);
                weight += std::exp(-0.5 * std::pow(distance_error_norm / cfg.measurement_noise, 2))
                          / (2 * M_PI * std::pow(cfg.measurement_noise, 2));
            }
            // If the observation is outside the max range, penalise it
            else {
                weight *= (1 - 1 / observations.size());
            }
        }

        return std::max(weight, 0.0);
    }

}  // namespace module::localisation
