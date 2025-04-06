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

#include "message/localisation/Robot.hpp"
#include "message/vision/GreenHorizon.hpp"
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

            // Set our UKF filter parameters
            cfg.ukf.noise.measurement.position =
                Eigen::Vector2d(config["ukf"]["noise"]["measurement"]["robot_position"].as<Expression>()).asDiagonal();
            cfg.ukf.noise.process.position      = config["ukf"]["noise"]["process"]["position"].as<Expression>();
            cfg.ukf.noise.process.velocity      = config["ukf"]["noise"]["process"]["velocity"].as<Expression>();
            cfg.ukf.initial_covariance.position = config["ukf"]["initial_covariance"]["position"].as<Expression>();
            cfg.ukf.initial_covariance.velocity = config["ukf"]["initial_covariance"]["velocity"].as<Expression>();
            cfg.association_distance            = config["association_distance"].as<double>();
            cfg.max_missed_count                = config["max_missed_count"].as<int>();
        });


        on<Trigger<VisionRobots>, With<GreenHorizon>, Single>().then(
            [this](const VisionRobots& vision_robots, const GreenHorizon& horizon) {
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

                // **Run maintenance step**
                maintenance(horizon);

                // **Debugging output**
                // Print tracked_robots ids
                log<DEBUG>("Robots tracked:");
                for (const auto& tracked_robot : tracked_robots) {
                    log<DEBUG>("\tID: ", tracked_robot.id);
                }

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
                    localisation_robots->robots.push_back(localisation_robot);
                }
                emit(std::move(localisation_robots));
            });
    }

    void RobotLocalisation::prediction() {
        auto now = NUClear::clock::now();

        for (auto& tracked_robot : tracked_robots) {
            double dt =
                std::chrono::duration_cast<std::chrono::duration<double>>(now - tracked_robot.last_time_update).count();
            tracked_robot.last_time_update = now;
            tracked_robot.ukf.time(dt);
            tracked_robot.seen = false;  // Reset 'seen' status
        }
    }

    void RobotLocalisation::data_association(const std::vector<Eigen::Vector3d>& robots_rRWw) {
        Eigen::Isometry3d Hwc = Eigen::Isometry3d(vision_robots.Hcw).inverse();

        for (const auto& robot : robots) {
            TrackedRobot* best_match = nullptr;
            double best_distance     = std::numeric_limits<double>::max();

            // Find closest existing robot
            for (auto& tracked_robot : tracked_robots) {
                double distance = (robot.rRWw.head<2>() - tracked_robot.get_rRWw()).norm();
                if (distance < cfg.association_distance && distance < best_distance) {
                    best_distance = distance;
                    best_match    = &tracked_robot;  // Use reference to the existing tracked robot
                }
            }

            if (best_match) {
                // Update matched robot with the vision measurement
                best_match->ukf.measure(Eigen::Vector2d(robot.rRWw.head<2>()),
                                        cfg.ukf.noise.measurement.position,
                                        MeasurementType::ROBOT_POSITION());
                best_match->seen = true;
            }
            else {
                // No close robot: start tracking a new one
                tracked_robots.emplace_back(TrackedRobot(robot.rRWw, cfg.ukf, next_id++));
                tracked_robots.back().seen = true;
            }
        }
    }

    void RobotLocalisation::maintenance(const GreenHorizon& horizon) {
        std::vector<TrackedRobot> new_tracked_robots{};

        for (auto& tracked_robot : tracked_robots) {
            auto state = RobotModel<double>::StateVec(tracked_robot.ukf.get_state());

            bool inside_view =
                point_in_convex_hull(horizon.horizon, Eigen::Vector3d(state.rRWw.x(), state.rRWw.y(), 0));

            if (!inside_view) {
                // If outside field of view, keep robot marked as 'seen' (prevent early deletion)
                tracked_robot.seen = true;
            }

            // Update missed counter
            tracked_robot.missed_count = tracked_robot.seen ? 0 : tracked_robot.missed_count + 1;

            // Keep only robots that have not been missed too many times
            if (tracked_robot.missed_count <= cfg.max_missed_count) {
                new_tracked_robots.push_back(std::move(tracked_robot));
            }
        }

        tracked_robots = std::move(new_tracked_robots);
    }

}  // namespace module::localisation
