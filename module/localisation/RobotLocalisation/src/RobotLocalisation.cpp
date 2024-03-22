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
            cfg.association_distance = config["association_distance"].as<double>();
            cfg.max_missed_count     = config["max_missed_count"].as<int>();
        });


        on<Trigger<VisionRobots>, With<GreenHorizon>, Single>().then([this](const VisionRobots& vision_robots,
                                                                            const GreenHorizon& horizon) {
            // Lock mutex to prevent multiple reactors from running at the same time
            std::lock_guard<std::mutex> lock(mutex);

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
                        auto tracked_robot = data_association(rRWw, tracked_robots);

                        log<NUClear::DEBUG>("Robot ",
                                            tracked_robot->id,
                                            " associated with vision measurement ",
                                            rRWw.transpose());
                    }
                }
            }

            // Robot tracking maintenance
            for (auto& tracked_robot : tracked_robots) {
                auto state = RobotModel<double>::StateVec(tracked_robot.ukf.get_state());

                // Consider a robot "seen" if it is not within the green horizon
                // TODO: It may be better to use fov and image size to determine if a robot should be seen
                if (!point_in_convex_hull(horizon.horizon, Eigen::Vector3d(state.rRWw.x(), state.rRWw.y(), 0))) {
                    tracked_robot.seen = true;
                }

                // If the tracked robot has not been seen, increment the consecutively missed count
                if (tracked_robot.seen) {
                    tracked_robot.missed_count = 0;
                }
                else {
                    tracked_robot.missed_count++;
                }
            }

            // Remove a robot if it is close to another robot
            tracked_robots.erase(
                std::remove_if(tracked_robots.begin(),
                               tracked_robots.end(),
                               [this](const TrackedRobot& tracked_robot) {
                                   bool remove = false;
                                   for (const auto& other_robot : tracked_robots) {
                                       if (tracked_robot.id != other_robot.id) {
                                           auto state = RobotModel<double>::StateVec(tracked_robot.ukf.get_state());
                                           auto other_state = RobotModel<double>::StateVec(other_robot.ukf.get_state());
                                           if ((state.rRWw - other_state.rRWw).norm() < cfg.association_distance) {
                                               remove = true;
                                               log<NUClear::WARN>("Removing robot due to close proximity");
                                               break;
                                           }
                                       }
                                   }
                                   return remove;
                               }),
                tracked_robots.end());

            // Remove a robot if it has been missed for too long
            tracked_robots.erase(std::remove_if(tracked_robots.begin(),
                                                tracked_robots.end(),
                                                [this](const TrackedRobot& tracked_robot) {
                                                    bool remove = tracked_robot.missed_count > cfg.max_missed_count;
                                                    if (remove) {
                                                        log<NUClear::WARN>("Removing robot due to missed count");
                                                    }
                                                    return remove;
                                                }),
                                 tracked_robots.end());


            // Emit the localisation of the robots
            auto localisation_robots = std::make_unique<LocalisationRobots>();
            // log<NUClear::DEBUG>("******************************************************\n");
            log<NUClear::DEBUG>("Number of tracked robots: ", tracked_robots.size());
            for (const auto& tracked_robot : tracked_robots) {
                auto state = RobotModel<double>::StateVec(tracked_robot.ukf.get_state());
                log<NUClear::DEBUG>("Robot ID: ",
                                    tracked_robot.id,
                                    " seen: ",
                                    tracked_robot.seen,
                                    " missed count: ",
                                    tracked_robot.missed_count,
                                    "rRWw: ",
                                    state.rRWw.x(),
                                    " ",
                                    state.rRWw.y());

                LocalisationRobot localisation_robot;
                localisation_robot.id                  = tracked_robot.id;
                localisation_robot.rRWw                = Eigen::Vector3d(state.rRWw.x(), state.rRWw.y(), 0);
                localisation_robot.vRw                 = Eigen::Vector3d(state.vRw.x(), state.vRw.y(), 0);
                localisation_robot.covariance          = tracked_robot.ukf.get_covariance();
                localisation_robot.time_of_measurement = tracked_robot.last_time_update;
                localisation_robots->robots.push_back(localisation_robot);
            }
            emit(std::move(localisation_robots));
        });
    }

    void RobotLocalisation::add_new_robot(const Eigen::Vector3d& initial_rRWw) {
        // Create a new robot
        // Set our motion model's process noise
        RobotModel<double>::StateVec process_noise;
        process_noise.rRWw = cfg.ukf.noise.process.position;
        process_noise.vRw  = cfg.ukf.noise.process.velocity;
        // Initialize the robot's state and noise
        RobotModel<double>::StateVec initial_state;
        initial_state.rRWw = initial_rRWw.head<2>();
        initial_state.vRw  = Eigen::Vector2d::Zero();

        // Set the robot's last time update to now
        // Set the robot as seen
        TrackedRobot new_tracked_robot(++robot_id,
                                       initial_state,
                                       cfg.initial_covariance,
                                       process_noise,
                                       NUClear::clock::now());
        new_tracked_robot.seen = true;
        log<NUClear::DEBUG>("Adding new robot with ID: ", new_tracked_robot.id);
        // Add the new robot to the list of tracked robots
        tracked_robots.push_back(new_tracked_robot);
    }

    std::shared_ptr<TrackedRobot> RobotLocalisation::data_association(const Eigen::Vector3d& rRWw,
                                                                      std::vector<TrackedRobot>& tracked_robots) {
        if (tracked_robots.empty()) {
            add_new_robot(rRWw);
            return std::make_shared<TrackedRobot>(tracked_robots.front());
        }

        double closest_distance                               = std::numeric_limits<double>::max();
        std::vector<TrackedRobot>::iterator closest_robot_itr = tracked_robots.begin();
        for (auto itr = tracked_robots.begin(); itr != tracked_robots.end(); ++itr) {
            auto tracked_robot_rRWw = RobotModel<double>::StateVec(itr->ukf.get_state()).rRWw;
            double current_distance = (rRWw.head<2>() - tracked_robot_rRWw).norm();
            if (current_distance < closest_distance) {
                closest_distance  = current_distance;
                closest_robot_itr = itr;
            }
        }

        if (closest_distance > cfg.association_distance) {
            add_new_robot(rRWw);
            return std::make_shared<TrackedRobot>(tracked_robots.back());
        }

        // Filter tracked robot associated with the vision measurement
        const auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now()
                                                                                  - closest_robot_itr->last_time_update)
                            .count();
        closest_robot_itr->last_time_update = NUClear::clock::now();
        closest_robot_itr->ukf.time(dt);
        closest_robot_itr->ukf.measure(Eigen::Vector2d(rRWw.head<2>()),
                                       cfg.ukf.noise.measurement.position,
                                       MeasurementType::ROBOT_POSITION());
        closest_robot_itr->seen = true;

        return std::make_shared<TrackedRobot>(*closest_robot_itr);
    }


}  // namespace module::localisation
