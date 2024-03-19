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
        });

        on<Trigger<VisionRobots>>().then([this](const VisionRobots& vision_robots) {
            if (!vision_robots.robots.empty()) {
                Eigen::Isometry3d Hwc = Eigen::Isometry3d(vision_robots.Hcw.cast<double>()).inverse();

                for (const auto& vision_robot : vision_robots.robots) {
                    // Position of robot in world space
                    auto rRWw = Hwc * vision_robot.rRCc;

                    // Data association: find the filtered robot which is best associated with the current vision robot
                    int id = data_association(rRWw, filtered_robots);

                    if (id == -1) {
                        // If no robot is associated, create a new one
                        id = add_new_robot(rRWw);
                        log<NUClear::DEBUG>("New robot added with id: ", id);
                    }
                    else {
                        log<NUClear::DEBUG>("Robot ", id, " associated with vision robot");
                    }

                    // Get the filtered robot associated with the current vision robot
                    auto& filtered_robot = filtered_robots[id];

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

                for (auto& filtered_robot : filtered_robots) {
                    auto state = RobotModel<double>::StateVec(filtered_robot.ukf.get_state());
                    emit(graph("Robot " + std::to_string(filtered_robot.id) + " rRWw", state.rRWw.x(), state.rRWw.y()));
                }


                // // Generate and emit message
                // auto robot                 = std::make_unique<Robot>();
                // robot->rRWw                = Eigen::Vector3d(state.rRWw.x(), state.rRWw.y(), fd.robot_radius);
                // robot->vRw                 = Eigen::Vector3d(state.vRw.x(), state.vRw.y(), 0);
                // robot->time_of_measurement = last_time_update;
                // robot->Hcw                 = robots.Hcw;
                // if (log_level <= NUClear::DEBUG) {
                //     log<NUClear::DEBUG>("rRWw: ", robot->rRWw.x(), robot->rRWw.y(), robot->rRWw.z());
                //     log<NUClear::DEBUG>("vRw: ", robot->vRw.x(), robot->vRw.y(), robot->vRw.z());
                //     emit(graph("rRWw: ", robot->rRWw.x(), robot->rRWw.y(), robot->rRWw.z()));
                //     emit(graph("vRw: ", robot->vRw.x(), robot->vRw.y(), robot->vRw.z()));
                // }

                // emit(robot);
            }
        });
    }

    int RobotLocalisation::add_new_robot(const Eigen::Vector3d& initial_rRWw) {
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
        // Add the new robot to the list of filtered robots
        filtered_robots.push_back(new_robot);
        return new_robot.id;
    }

    int RobotLocalisation::data_association(const Eigen::Vector3d& rRWw,
                                            const std::vector<FilteredRobot>& filtered_robots) {
        // If no filtered robots, return -1
        int id = -1;
        if (filtered_robots.empty()) {
            return id;
        }

        // Find the filtered robot which is closest to the current vision robot
        // Data association: find the ball closest to our current estimate
        double lowest_distance = std::numeric_limits<double>::max();
        for (const auto& filtered_robot : filtered_robots) {
            auto filtered_robot_rRWw = RobotModel<double>::StateVec(filtered_robot.ukf.get_state()).rRWw;
            double current_distance  = (rRWw.head<2>() - filtered_robot_rRWw).squaredNorm();
            if (current_distance < lowest_distance) {
                lowest_distance = current_distance;
                id              = filtered_robot.id;
            }
        }

        // TODO: If the distance is too large, return -1

        return id;
    }

}  // namespace module::localisation
