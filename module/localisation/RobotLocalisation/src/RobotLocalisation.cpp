#include "RobotLocalisation.hpp"

#include "extension/Configuration.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::localisation {

    using extension::Configuration;

    using message::eye::DataPoint;

    using utility::nusight::graph;
    using utility::support::Expression;


    RobotLocalisation::RobotLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("RobotLocalisation.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RobotLocalisation.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // Set our measurement noise
            cfg.ukf.noise.measurement.position =
                Eigen::Vector2d(config["ukf"]["noise"]["measurement"]["robot_position"].as<Expression>()).asDiagonal();

            // Set our process noises
            cfg.ukf.noise.process.position = config["ukf"]["noise"]["process"]["position"].as<Expression>();
            cfg.ukf.noise.process.velocity = config["ukf"]["noise"]["process"]["velocity"].as<Expression>();

            // Set our initial mean
            cfg.ukf.initial.mean.position = config["ukf"]["initial"]["mean"]["position"].as<Expression>();
            cfg.ukf.initial.mean.velocity = config["ukf"]["initial"]["mean"]["velocity"].as<Expression>();

            // Set out initial covariance
            cfg.ukf.initial.covariance.position = config["ukf"]["initial"]["covariance"]["position"].as<Expression>();
            cfg.ukf.initial.covariance.velocity = config["ukf"]["initial"]["covariance"]["velocity"].as<Expression>();

            // Set our initial state with the config means and covariances, flagging the filter to reset it
            cfg.initial_mean.rRWw = cfg.ukf.initial.mean.position;
            cfg.initial_mean.vRw  = cfg.ukf.initial.mean.velocity;

            // Set our max association distance and max missed count
            cfg.max_association_distance = config["max_association_distance"].as<double>();
            cfg.max_missed_count         = config["max_missed_count"].as<int>();
        });

        on<Trigger<VisionRobots>>().then([this](const VisionRobots& vision_robots) {
            if (!vision_robots.robots.empty()) {
                Eigen::Isometry3d Hwc = Eigen::Isometry3d(vision_robots.Hcw.cast<double>()).inverse();

                // Set all filtered robots to unseen
                for (auto& filtered_robot : filtered_robots) {
                    filtered_robot.seen = false;
                }

                for (const auto& vision_robot : vision_robots.robots) {
                    // Position of robot in world space
                    auto rRWw = Hwc * vision_robot.rRCc;

                    // Data association: find the filtered robot which is best associated with the current vision robot
                    auto& filtered_robot = data_association(rRWw, filtered_robots);

                    // Perform the time and measurement update
                    const auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                                        NUClear::clock::now() - filtered_robot.last_time_update)
                                        .count();
                    filtered_robot.last_time_update = NUClear::clock::now();

                    filtered_robot.ukf.time(dt);
                    filtered_robot.ukf.measure(Eigen::Vector2d(rRWw.head<2>()),
                                               cfg.ukf.noise.measurement.position,
                                               MeasurementType::ROBOT_POSITION());
                }

                // If a robot has not been seen for a certain number of frames, remove it
                for (auto& filtered_robot : filtered_robots) {
                    if (!filtered_robot.seen) {
                        filtered_robot.missed_count++;
                    }
                    else {
                        filtered_robot.missed_count = std::max(0, filtered_robot.missed_count - 1);
                    }
                }
                filtered_robots.erase(std::remove_if(filtered_robots.begin(),
                                                     filtered_robots.end(),
                                                     [this](const FilteredRobot& filtered_robot) {
                                                         return filtered_robot.missed_count > cfg.max_missed_count;
                                                     }),
                                      filtered_robots.end());


                // Emit the localisation of the robots
                auto localisation_robots = std::make_unique<LocalisationRobots>();
                for (auto& filtered_robot : filtered_robots) {
                    auto state = RobotModel<double>::StateVec(filtered_robot.ukf.get_state());
                    LocalisationRobot localisation_robot;
                    localisation_robot.id                  = filtered_robot.id;
                    localisation_robot.rRWw                = Eigen::Vector3d(state.rRWw.x(), state.rRWw.y(), 0);
                    localisation_robot.vRw                 = Eigen::Vector3d(state.vRw.x(), state.vRw.y(), 0);
                    localisation_robot.covariance          = filtered_robot.ukf.get_covariance();
                    localisation_robot.time_of_measurement = filtered_robot.last_time_update;
                    localisation_robots->robots.push_back(localisation_robot);
                    emit(graph("Robot " + std::to_string(filtered_robot.id) + " rRWw", state.rRWw.x(), state.rRWw.y()));
                }
                emit(std::move(localisation_robots));
            }
        });
    }

    void RobotLocalisation::add_new_robot(const Eigen::Vector3d& initial_rRWw) {
        // Create a new robot
        FilteredRobot new_robot;
        new_robot.id = filtered_robots.size();
        // Set our motion model's process noise
        RobotModel<double>::StateVec process_noise;
        process_noise.rRWw                = cfg.ukf.noise.process.position;
        process_noise.vRw                 = cfg.ukf.noise.process.velocity;
        new_robot.ukf.model.process_noise = process_noise;
        // Initialize the robot's state and noise
        cfg.initial_mean.rRWw = initial_rRWw.head<2>();
        new_robot.ukf.set_state(cfg.initial_mean.getStateVec(), cfg.initial_covariance.asDiagonal());
        // Set the robot's last time update to now
        new_robot.last_time_update = NUClear::clock::now();
        // Set the robot as seen
        new_robot.seen = true;
        // Add the new robot to the list of filtered robots
        log<NUClear::DEBUG>("Adding new robot: ", new_robot.id);
        filtered_robots.push_back(new_robot);
    }

    RobotLocalisation::FilteredRobot& RobotLocalisation::data_association(const Eigen::Vector3d& rRWw,
                                                                          std::vector<FilteredRobot>& filtered_robots) {
        // If no filtered robots, add a new robot to the filtered robot list
        if (filtered_robots.empty()) {
            add_new_robot(rRWw);
            return filtered_robots.back();
        }

        // Find the filtered robot which is closest to the current vision robot
        double closest_distance      = std::numeric_limits<double>::max();
        FilteredRobot& closest_robot = filtered_robots.front();
        for (const auto& filtered_robot : filtered_robots) {
            auto filtered_robot_rRWw = RobotModel<double>::StateVec(filtered_robot.ukf.get_state()).rRWw;
            double current_distance  = (rRWw.head<2>() - filtered_robot_rRWw).squaredNorm();
            if (current_distance < closest_distance) {
                closest_distance = current_distance;
                closest_robot    = filtered_robot;
            }
        }

        // If the distance is too large, add a new robot to the filtered robot list
        if (closest_distance > cfg.max_association_distance) {
            log<NUClear::DEBUG>("Robot detection ",
                                std::to_string(closest_distance),
                                " m away from the closest robot, adding new robot");
            add_new_robot(rRWw);
            return filtered_robots.back();
        }

        // Set the associated robot as seen
        closest_robot.seen = true;
        return closest_robot;
    }

}  // namespace module::localisation
