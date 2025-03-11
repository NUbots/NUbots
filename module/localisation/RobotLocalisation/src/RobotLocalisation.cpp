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

#include "message/localisation/Field.hpp"
#include "message/localisation/Robot.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/GreenHorizon.hpp"
#include "message/vision/Robot.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/visualmesh/VisualMesh.hpp"

namespace module::localisation {

    using extension::Configuration;

    using LocalisationRobot  = message::localisation::Robot;
    using LocalisationRobots = message::localisation::Robots;
    using LocalisationField  = message::localisation::Field;
    using VisionRobot        = message::vision::Robot;
    using VisionRobots       = message::vision::Robots;
    using FieldDescription   = message::support::FieldDescription;

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


        on<Trigger<VisionRobots>, With<GreenHorizon>, With<FieldDescription>, With<LocalisationField>, Single>().then(
            [this](const VisionRobots& vision_robots,
                   const GreenHorizon& horizon,
                   const FieldDescription& field_description,
                   const LocalisationField& localisation_field) {
                // Print tracked_robots ids
                log<DEBUG>("Robots tracked:");
                for (const auto& tracked_robot : tracked_robots) {
                    log<DEBUG>("\tID: ", tracked_robot.id);
                }

                // Set all tracked robots to unseen
                for (auto& tracked_robot : tracked_robots) {
                    tracked_robot.seen = false;
                }

                // **************** Data association ****************
                if (!vision_robots.robots.empty()) {
                    Eigen::Isometry3d Hwc = Eigen::Isometry3d(vision_robots.Hcw).inverse();
                    for (const auto& vision_robot : vision_robots.robots) {
                        // Position of robot {r} in world {w} space
                        auto rRWw = Hwc * vision_robot.rRCc;

                        // Data association: find tracked robot which is associated with the vision measurement
                        data_association(rRWw);
                    }
                }

                // **************** Robot tracking maintenance ****************
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
                }

                // Make a vector to store kept robots
                std::vector<TrackedRobot> new_tracked_robots;
                // Only keep robots that are not missing or too close to others
                for (const auto& tracked_robot : tracked_robots) {
                    if (tracked_robot.missed_count > cfg.max_missed_count) {
                        log<DEBUG>(fmt::format("Removing robot {} due to missed count", tracked_robot.id));
                        continue;
                    }

                    if (std::any_of(tracked_robots.begin(), tracked_robots.end(), [&](const auto& other_robot) {
                            return &tracked_robot != &other_robot
                                   && (tracked_robot.get_rRWw() - other_robot.get_rRWw()).norm()
                                          < cfg.association_distance;
                        })) {
                        log<DEBUG>(fmt::format("Removing robot {} due to proximity", tracked_robot.id));
                        continue;
                    }

                    Eigen::Vector3d rRFf =
                        localisation_field.Hfw
                        * Eigen::Vector3d(tracked_robot.get_rRWw().x(), tracked_robot.get_rRWw().y(), 0);

                    if ((rRFf.x() < field_description.dimensions.field_length / 2
                         || rRFf.x() > -field_description.dimensions.field_length / 2)
                        && (rRFf.y() < field_description.dimensions.field_width / 2
                            || rRFf.y() > -field_description.dimensions.field_width / 2)) {
                        log<DEBUG>(fmt::format("Removing robot {} due to location (outside field)", tracked_robot.id));
                        continue;
                    }

                    // If neither case is true, keep the robot
                    new_tracked_robots.push_back(tracked_robot);
                }
                tracked_robots = std::move(new_tracked_robots);

                // Emit the localisation of the robots
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

    void RobotLocalisation::data_association(const Eigen::Vector3d& rRWw) {
        // If we have no robots yet, this must be a new robot
        if (tracked_robots.empty()) {
            tracked_robots.emplace_back(TrackedRobot(rRWw, cfg.ukf, next_id++));
            return;
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
            tracked_robots.emplace_back(TrackedRobot(rRWw, cfg.ukf, next_id++));
            return;
        }

        // Update the filter on the robot associated with the vision measurement
        auto now = NUClear::clock::now();
        const auto dt =
            std::chrono::duration_cast<std::chrono::duration<double>>(now - closest_robot_itr->last_time_update)
                .count();
        closest_robot_itr->last_time_update = now;
        closest_robot_itr->ukf.time(dt);
        closest_robot_itr->ukf.measure(Eigen::Vector2d(rRWw.head<2>()),
                                       cfg.ukf.noise.measurement.position,
                                       MeasurementType::ROBOT_POSITION());
        closest_robot_itr->seen = true;
    }


}  // namespace module::localisation
