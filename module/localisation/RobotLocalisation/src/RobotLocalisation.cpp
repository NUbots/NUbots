#include "RobotLocalisation.hpp"

#include "extension/Configuration.hpp"

#include "message/vision/GreenHorizon.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/visualmesh/VisualMesh.hpp"

namespace module::localisation {

    using extension::Configuration;

    using message::eye::DataPoint;

    using message::vision::GreenHorizon;

    using utility::math::geometry::point_in_convex_hull;

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
            cfg.min_seen_count           = config["min_seen_count"].as<int>();
        });


        on<Trigger<VisionRobots>, With<GreenHorizon>, Single>().then(
            [this](const VisionRobots& vision_robots, const GreenHorizon& horizon) {
                // Set all tracked robots to unseen
                for (auto& tracked_robot : tracked_robots) {
                    tracked_robot.seen = false;
                }

                if (!vision_robots.robots.empty()) {
                    Eigen::Isometry3d Hwc = Eigen::Isometry3d(vision_robots.Hcw).inverse();

                    for (const auto& vision_robot : vision_robots.robots) {
                        // Position of robot {r} in world {w} space
                        auto rRWw = Hwc * vision_robot.rRCc;

                        // Only consider vision measurements within the green horizon
                        if (point_in_convex_hull(horizon.horizon, rRWw)) {
                            // Data association: find tracked robot which is associated with the vision measurement
                            auto& tracked_robot = data_association(rRWw, tracked_robots);

                            // Filter tracked robot associated with the vision measurement
                            const auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                                                NUClear::clock::now() - tracked_robot.last_time_update)
                                                .count();
                            tracked_robot.last_time_update = NUClear::clock::now();
                            tracked_robot.ukf.time(dt);
                            tracked_robot.ukf.measure(Eigen::Vector2d(rRWw.head<2>()),
                                                      cfg.ukf.noise.measurement.position,
                                                      MeasurementType::ROBOT_POSITION());
                        }
                    }
                }

                // Robot tracking maintenance
                for (auto& tracked_robot : tracked_robots) {
                    auto state = RobotModel<double>::StateVec(tracked_robot.ukf.get_state());

                    // If the tracked robot has been seen, increment the consecutively seen count
                    if (tracked_robot.seen) {
                        tracked_robot.seen_count++;
                    }
                    else {
                        tracked_robot.seen_count = 0;
                    }

                    // If the tracked robot has been seen enough times, confirm it
                    if (tracked_robot.seen_count > cfg.min_seen_count) {
                        tracked_robot.confirmed = true;
                    }

                    // Consider a robot "seen" if it is not within the green horizon
                    // TODO: It may be better to use fov and image size to determine if a robot should be seen
                    if (!point_in_convex_hull(horizon.horizon, Eigen::Vector3d(state.rRWw.x(), state.rRWw.y(), 0))) {
                        tracked_robot.seen = true;
                    }

                    // If the tracked robot has not been seen, increment the consecutively missed count
                    if (!tracked_robot.seen) {
                        tracked_robot.missed_count++;
                    }
                    else {
                        tracked_robot.missed_count = 0;
                    }
                }
                tracked_robots.erase(std::remove_if(tracked_robots.begin(),
                                                    tracked_robots.end(),
                                                    [this](const TrackedRobot& tracked_robot) {
                                                        return tracked_robot.missed_count > cfg.max_missed_count;
                                                    }),
                                     tracked_robots.end());

                // Emit the localisation of the robots
                auto localisation_robots = std::make_unique<LocalisationRobots>();
                log<NUClear::DEBUG>("******************************************************\n");
                log<NUClear::DEBUG>("Number of tracked robots: ", tracked_robots.size());
                for (const auto& tracked_robot : tracked_robots) {
                    log<NUClear::DEBUG>("Robot ID: ",
                                        tracked_robot.id,
                                        " confirmed: ",
                                        tracked_robot.confirmed,
                                        " seen: ",
                                        tracked_robot.seen,
                                        " seen count: ",
                                        tracked_robot.seen_count,
                                        " missed count: ",
                                        tracked_robot.missed_count);


                    if (tracked_robot.confirmed) {
                        auto state = RobotModel<double>::StateVec(tracked_robot.ukf.get_state());
                        LocalisationRobot localisation_robot;
                        localisation_robot.id                  = tracked_robot.id;
                        localisation_robot.rRWw                = Eigen::Vector3d(state.rRWw.x(), state.rRWw.y(), 0);
                        localisation_robot.vRw                 = Eigen::Vector3d(state.vRw.x(), state.vRw.y(), 0);
                        localisation_robot.covariance          = tracked_robot.ukf.get_covariance();
                        localisation_robot.time_of_measurement = tracked_robot.last_time_update;
                        localisation_robots->robots.push_back(localisation_robot);
                    }
                }
                emit(std::move(localisation_robots));
            });
    }

    void RobotLocalisation::add_new_robot(const Eigen::Vector3d& initial_rRWw) {
        // Create a new robot
        TrackedRobot new_tracked_robot;
        new_tracked_robot.id = robot_id;
        robot_id++;
        // Set our motion model's process noise
        RobotModel<double>::StateVec process_noise;
        process_noise.rRWw                        = cfg.ukf.noise.process.position;
        process_noise.vRw                         = cfg.ukf.noise.process.velocity;
        new_tracked_robot.ukf.model.process_noise = process_noise;
        // Initialize the robot's state and noise
        cfg.initial_mean.rRWw = initial_rRWw.head<2>();
        new_tracked_robot.ukf.set_state(cfg.initial_mean.getStateVec(), cfg.initial_covariance.asDiagonal());
        // Set the robot's last time update to now
        new_tracked_robot.last_time_update = NUClear::clock::now();
        // Set the robot as seen
        new_tracked_robot.seen = true;
        // Add the new robot to the list of tracked robots
        tracked_robots.push_back(new_tracked_robot);
    }

    RobotLocalisation::TrackedRobot& RobotLocalisation::data_association(const Eigen::Vector3d& rRWw,
                                                                         std::vector<TrackedRobot>& tracked_robots) {
        // If no tracked robots, add first tracked robot to list
        if (tracked_robots.empty()) {
            add_new_robot(rRWw);
            return tracked_robots.back();
        }

        // Find the tracked robot which is closest to the current robot vision estimate
        // TODO: This is a naive approach to data association, a more sophisticated approach should be used
        // Options include nearest neighbour with mahalanobis distance, Joint Probabilistic Data Association, etc.
        double closest_distance     = std::numeric_limits<double>::max();
        TrackedRobot& closest_robot = tracked_robots.front();
        for (const auto& tracked_robot : tracked_robots) {
            auto tracked_robot_rRWw = RobotModel<double>::StateVec(tracked_robot.ukf.get_state()).rRWw;
            double current_distance = (rRWw.head<2>() - tracked_robot_rRWw).norm();
            if (current_distance < closest_distance) {
                closest_distance = current_distance;
                closest_robot    = tracked_robot;
            }
        }

        // If the distance is too large, add a new robot to the tracked robot list associated with the measurement
        if (closest_distance > cfg.max_association_distance) {
            add_new_robot(rRWw);
            return tracked_robots.back();
        }

        // Set the associated robot as seen
        closest_robot.seen = true;
        return closest_robot;
    }

}  // namespace module::localisation
