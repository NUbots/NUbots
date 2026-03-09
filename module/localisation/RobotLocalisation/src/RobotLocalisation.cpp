/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
#include "RobotLocalisation.hpp"

#include <fmt/format.h>

#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/input/RoboCup.hpp"
#include "message/localisation/Robot.hpp"
#include "message/localisation/Swarm.hpp"
#include "message/vision/Robot.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/visualmesh/VisualMesh.hpp"

namespace module::localisation {

    using extension::Configuration;

    using LocalisationRobot  = message::localisation::Robot;
    using LocalisationRobots = message::localisation::Robots;
    using VisionRobot        = message::vision::Robot;
    using VisionRobots       = message::vision::Robots;
    using PenaltyState       = message::input::State;

    using message::eye::DataPoint;
    using message::input::GameState;
    using message::input::RoboCup;
    using message::localisation::Field;
    using message::localisation::SwarmState;
    using message::purpose::Purpose;
    using message::purpose::SoccerPosition;
    using message::support::FieldDescription;
    using message::vision::GreenHorizon;

    using utility::math::geometry::point_in_convex_hull;
    using utility::nusight::graph;
    using utility::support::Expression;

    RobotLocalisation::RobotLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("RobotLocalisation.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RobotLocalisation.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // Set our UKF filter parameters
            cfg.ukf.noise.measurement.position =
                Eigen::Vector2d(config["ukf"]["noise"]["measurement"]["robot_position"].as<Expression>()).asDiagonal();
            cfg.ukf.noise.process.position      = config["ukf"]["noise"]["process"]["position"].as<Expression>();
            cfg.ukf.noise.process.velocity      = config["ukf"]["noise"]["process"]["velocity"].as<Expression>();
            cfg.ukf.initial_covariance.position = config["ukf"]["initial_covariance"]["position"].as<Expression>();
            cfg.ukf.initial_covariance.velocity = config["ukf"]["initial_covariance"]["velocity"].as<Expression>();
            cfg.association_distance            = config["association_distance"].as<double>();
            cfg.max_missed_count                = config["max_missed_count"].as<int>();
            cfg.max_distance_from_field         = config["max_distance_from_field"].as<double>();
            cfg.max_localisation_cost           = config["max_localisation_cost"].as<double>();
            cfg.max_opponents                   = config["max_opponents"].as<int>();
        });

        on<Every<UPDATE_RATE, Per<std::chrono::seconds>>,
           With<GreenHorizon>,
           With<Field>,
           With<FieldDescription>,
           Optional<With<SwarmState>>,
           Sync<RobotLocalisation>>()
            .then([this](const GreenHorizon& horizon,
                         const Field& field,
                         const FieldDescription& field_desc,
                         const std::shared_ptr<const SwarmState>& swarm_state) {
                // **Run maintenance step**
                maintenance(horizon, field, field_desc);

                // **Debugging output**
                debug_info();

                // **Emit the localisation of the robots**
                auto localisation_robots = std::make_unique<LocalisationRobots>();
                for (const auto& tracked_robot : tracked_robots) {
                    auto state = RobotModel<double>::StateVec(tracked_robot.ukf.get_state());
                    LocalisationRobot localisation_robot;
                    localisation_robot.id                  = tracked_robot.id;
                    localisation_robot.rRWw                = Eigen::Vector3d(state.rRWw.x(), state.rRWw.y(), 0);
                    localisation_robot.vRw                 = Eigen::Vector3d(state.vRw.x(), state.vRw.y(), 0);
                    localisation_robot.covariance          = tracked_robot.ukf.get_covariance();
                    localisation_robot.time_of_measurement = tracked_robot.last_time_update;
                    localisation_robot.teammate            = tracked_robot.teammate;
                    localisation_robot.purpose             = tracked_robot.purpose;
                    localisation_robot.from_swarm          = false;
                    localisation_robots->robots.push_back(localisation_robot);
                }

                // Append swarm-fused opponents for positions outside this robot's field of view.
                // Anything inside the green horizon is already handled by this robot's own vision —
                // swarm entries only fill blind spots, avoiding any association ambiguity.
                if (swarm_state && !horizon.horizon.empty()) {
                    for (const auto& opp_Ff : swarm_state->opponent_positions_Ff) {
                        Eigen::Vector3d rOFf(opp_Ff.x(), opp_Ff.y(), 0.0);
                        Eigen::Vector3d rOWw = field.Hfw.inverse() * rOFf;
                        if (!point_in_convex_hull(horizon.horizon, rOWw)) {
                            LocalisationRobot swarm_robot;
                            swarm_robot.rRWw       = rOWw;
                            swarm_robot.teammate   = false;
                            swarm_robot.from_swarm = true;
                            localisation_robots->robots.push_back(swarm_robot);
                        }
                    }
                }

                emit(std::move(localisation_robots));
            });

        on<Trigger<RoboCup>, With<Field>, Sync<RobotLocalisation>>().then(
            [this](const RoboCup& robocup, const Field& field) {
                // Do not consider a teammate's localisation if their cost is too high
                if (robocup.current_pose.cost > cfg.max_localisation_cost) {
                    log<DEBUG>("Teammate's localisation cost is too high, not processing.");
                    return;
                }

                // **Run prediction step**
                prediction();

                // **Data association**
                // RoboCup messages come from teammates. Their position is in field space, so convert to world.
                std::vector<Eigen::Vector3d> robots_rRWw{
                    (field.Hfw.inverse() * robocup.current_pose.position.cast<double>())};
                auto purpose = std::make_unique<Purpose>(robocup.purpose);
                // Run data association step
                data_association(robots_rRWw, purpose);
            });

        on<Trigger<VisionRobots>, Sync<RobotLocalisation>>().then([this](const VisionRobots& vision_robots) {
            // **Run prediction step**
            prediction();

            // **Data association**
            // Transform the robot positions from camera coordinates to world coordinates
            Eigen::Isometry3d Hwc = Eigen::Isometry3d(vision_robots.Hcw).inverse();
            // Make a vector with the transformed position
            std::vector<Eigen::Vector3d> robots_rRWw{};
            for (const auto& vision_robot : vision_robots.robots) {
                Eigen::Vector3d rRWw = Hwc * vision_robot.rRCc;
                robots_rRWw.push_back(rRWw);
            }
            // Run data association step
            data_association(robots_rRWw);
        });

    }

    void RobotLocalisation::prediction() {
        auto now = NUClear::clock::now();

        for (auto& tracked_robot : tracked_robots) {
            double dt =
                std::chrono::duration_cast<std::chrono::duration<double>>(now - tracked_robot.last_time_update).count();
            tracked_robot.last_time_update = now;
            tracked_robot.ukf.time(dt);
            tracked_robot.seen = false;
        }
    }

    void RobotLocalisation::data_association(const std::vector<Eigen::Vector3d>& robots_rRWw,
                                             const std::unique_ptr<Purpose>& purpose,
                                             bool refresh_seen) {
        for (const auto& rRWw : robots_rRWw) {
            if (tracked_robots.empty()) {
                // If there are no tracked robots, add this as a new robot
                tracked_robots.emplace_back(TrackedRobot(rRWw, cfg.ukf, next_id++, purpose));
                continue;
            }

            // If this is a teammate, find it in the list of tracked robots
            // If it doesn't exist, add it. Purpose is checked but should always be set for teammates.
            if (purpose) {
                // Check if the robot is already in the list of tracked robots
                auto teammate_itr =
                    std::find_if(tracked_robots.begin(), tracked_robots.end(), [&purpose](const TrackedRobot& robot) {
                        return robot.purpose.player_id == purpose->player_id;
                    });

                // If the robot is not in the list, add it
                if (teammate_itr == tracked_robots.end()) {
                    tracked_robots.emplace_back(TrackedRobot(rRWw, cfg.ukf, next_id++, purpose));
                    continue;
                }
                // If the robot is in the list, update it with the new position
                teammate_itr->ukf.measure(Eigen::Vector2d(rRWw.head<2>()),
                                          cfg.ukf.noise.measurement.position,
                                          MeasurementType::ROBOT_POSITION());
                teammate_itr->seen    = teammate_itr->seen || refresh_seen;
                teammate_itr->purpose = *purpose;

                continue;
            }

            // Get the closest robot we have to the given vision measurement
            auto closest_robot_itr =
                std::min_element(tracked_robots.begin(),
                                 tracked_robots.end(),
                                 [&rRWw](const TrackedRobot& a, const TrackedRobot& b) {
                                     // Get each robot's x-y 2D position in the world
                                     auto a_rRWw = a.get_rRWw();
                                     auto b_rRWw = b.get_rRWw();
                                     // Compare to see which is closer to the robot vision measurement position
                                     return (rRWw.head<2>() - a_rRWw).norm() < (rRWw.head<2>() - b_rRWw).norm();
                                 });
            double closest_distance = (rRWw.head<2>() - closest_robot_itr->get_rRWw()).norm();

            // If the closest robot is far enough away, add this as a new robot
            if (closest_distance > cfg.association_distance) {
                tracked_robots.emplace_back(TrackedRobot(rRWw, cfg.ukf, next_id++, purpose));
                continue;
            }

            // Otherwise update matched robot with the vision measurement
            closest_robot_itr->ukf.measure(Eigen::Vector2d(rRWw.head<2>()),
                                           cfg.ukf.noise.measurement.position,
                                           MeasurementType::ROBOT_POSITION());
            closest_robot_itr->seen = closest_robot_itr->seen || refresh_seen;
        }
    }

    void RobotLocalisation::maintenance(const GreenHorizon& horizon,
                                        const Field& field,
                                        const FieldDescription& field_desc) {
        std::vector<TrackedRobot> new_tracked_robots{};

        // Sort tracked_robots so that robots that are teammates are at the front to prevent teammates being removed
        std::sort(tracked_robots.begin(), tracked_robots.end(), [](const TrackedRobot& a, const TrackedRobot& b) {
            return a.purpose.player_id > b.purpose.player_id;
        });

        for (auto& tracked_robot : tracked_robots) {
            auto state = RobotModel<double>::StateVec(tracked_robot.ukf.get_state());

            // If a tracked robot has moved outside of view, keep it as seen so we don't lose it
            // A robot is outside of view if it is not within the green horizon
            // TODO (tom): It may be better to use fov and image size to determine if a robot should be seen
            if (!point_in_convex_hull(horizon.horizon, Eigen::Vector3d(state.rRWw.x(), state.rRWw.y(), 0))) {
                tracked_robot.seen = true;
            }

            // If the tracked robot has not been seen, increment the consecutively missed count
            tracked_robot.missed_count = tracked_robot.seen ? 0 : tracked_robot.missed_count + 1;

            // Don't add this robot if it has been missed too many times
            if (tracked_robot.missed_count > cfg.max_missed_count) {
                log<DEBUG>(fmt::format("Removing robot {} due to missed count", tracked_robot.id));
                continue;
            }

            // Check if this robot is too close to any kept robot
            if (std::any_of(new_tracked_robots.begin(), new_tracked_robots.end(), [&](const auto& other_robot) {
                    return (tracked_robot.get_rRWw() - other_robot.get_rRWw()).norm() < cfg.association_distance;
                })) {
                log<DEBUG>(fmt::format("Removing robot {} due to proximity", tracked_robot.id));
                continue;
            }

            Eigen::Vector3d rRFf = field.Hfw * Eigen::Vector3d(state.rRWw.x(), state.rRWw.y(), 0);

            if (rRFf.x() < (-field_desc.dimensions.field_length / 2) - cfg.max_distance_from_field
                || rRFf.x() > (field_desc.dimensions.field_length / 2) + cfg.max_distance_from_field
                || rRFf.y() < (-field_desc.dimensions.field_width / 2) - cfg.max_distance_from_field
                || rRFf.y() > (field_desc.dimensions.field_width / 2) + cfg.max_distance_from_field) {
                log<DEBUG>(fmt::format("Removing robot {} due to location (outside field)", tracked_robot.id));
                continue;
            }

            // If removal conditions not met, keep the robot
            new_tracked_robots.push_back(tracked_robot);
        }

        // Hard cap on the number of tracked opponents. Keep the most recently seen ones
        // (lowest missed_count). Teammates are always kept regardless of the cap.
        int opponent_count = static_cast<int>(
            std::count_if(new_tracked_robots.begin(), new_tracked_robots.end(), [](const TrackedRobot& r) {
                return !r.teammate;
            }));

        if (opponent_count > cfg.max_opponents) {
            // Stable-sort so teammates float to the front, opponents sorted by missed_count ascending
            std::stable_sort(new_tracked_robots.begin(), new_tracked_robots.end(), [](const TrackedRobot& a, const TrackedRobot& b) {
                if (a.teammate != b.teammate) {
                    return a.teammate > b.teammate;  // teammates first
                }
                return a.missed_count < b.missed_count;  // then lowest missed_count first
            });

            // Erase excess opponents from the back
            int to_remove = opponent_count - cfg.max_opponents;
            int removed   = 0;
            for (auto it = new_tracked_robots.end(); it != new_tracked_robots.begin() && removed < to_remove;) {
                --it;
                if (!it->teammate) {
                    log<DEBUG>(fmt::format("Removing robot {} due to opponent cap", it->id));
                    it = new_tracked_robots.erase(it);
                    ++removed;
                }
            }
        }

        tracked_robots = std::move(new_tracked_robots);
    }

    void RobotLocalisation::debug_info() const {
        // Print tracked_robots ids
        log<DEBUG>("Robots tracked:");
        for (const auto& tracked_robot : tracked_robots) {
            log<DEBUG>("\tID: ",
                       tracked_robot.id,
                       ": ",
                       tracked_robot.teammate ? "Teammate" : "Opponent",
                       "teammate ID: ",
                       tracked_robot.purpose.player_id,
                       "position: ",
                       RobotModel<double>::StateVec(tracked_robot.ukf.get_state()).rRWw.transpose());
        }
    }

}  // namespace module::localisation
