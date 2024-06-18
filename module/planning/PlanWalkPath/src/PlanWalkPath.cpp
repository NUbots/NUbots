/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#include "PlanWalkPath.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/Robot.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/skill/Kick.hpp"
#include "message/skill/Walk.hpp"

#include "utility/math/comparison.hpp"

namespace module::planning {

    using extension::Configuration;

    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::localisation::Robots;
    using message::planning::TurnAroundBall;
    using message::planning::TurnOnSpot;
    using message::planning::WalkTo;
    using message::skill::Walk;

    double g(const double x, const double y, const double x0, const double y0) {
        return 1 / std::sqrt(std::pow(x - x0, 2) + std::pow(y - y0, 2));
    }

    // https://www.desmos.com/calculator/exmrdiugxn
    Eigen::Vector2d PlanWalkPath::vector_field(const Eigen::Vector2d& self_position,
                                               const Eigen::Vector2d& target_position,
                                               const double heading,
                                               std::vector<Eigen::Vector2d> obstacles) {
        double x = self_position.x();
        double y = self_position.y();

        // Attraction of the target position
        Eigen::Vector2d vector_field_direction(
            (target_position.x() - x) * g(x, y, target_position.x(), target_position.y()) * cfg.target_strength,
            (target_position.y() - y) * g(x, y, target_position.x(), target_position.y()) * cfg.target_strength);

        // Create point next to target position at given heading
        double x_target = target_position.x() + std::cos(heading);
        double y_target = target_position.y() + std::sin(heading);

        // Repulse the heading point
        vector_field_direction.x() += ((x - x_target) * g(x, y, x_target, y_target)) * cfg.heading_strength;
        vector_field_direction.y() += ((y - y_target) * g(x, y, x_target, y_target)) * cfg.heading_strength;

        // Repulsion of the obstacles
        for (const auto& obstacle : obstacles) {
            vector_field_direction.x() +=
                ((x - obstacle.x()) * g(x, y, obstacle.x(), obstacle.y())) * cfg.obstacle_strength;
            vector_field_direction.y() +=
                ((y - obstacle.y()) * g(x, y, obstacle.x(), obstacle.y())) * cfg.obstacle_strength;
        }

        // Repulsion of the walls
        vector_field_direction.x() -=
            cfg.bounds_strength
            * (std::abs(x - 4.5) * utility::math::sgn(x - 4.5) + std::abs(x + 4.5) * utility::math::sgn(x + 4.5));
        vector_field_direction.y() -=
            cfg.bounds_strength
            * (std::abs(y - 3.0) * utility::math::sgn(y - 3.0) + std::abs(y + 3.0) * utility::math::sgn(y + 3.0));

        return vector_field_direction;
    }

    PlanWalkPath::PlanWalkPath(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanWalkPath.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanWalkPath.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.max_translational_velocity_magnitude = config["max_translational_velocity_magnitude"].as<double>();
            cfg.min_translational_velocity_magnitude = config["min_translational_velocity_magnitude"].as<double>();
            cfg.acceleration                         = config["acceleration"].as<double>();
            cfg.approach_radius                      = config["approach_radius"].as<double>();

            cfg.max_angular_velocity = config["max_angular_velocity"].as<double>();
            cfg.min_angular_velocity = config["min_angular_velocity"].as<double>();

            cfg.rotate_velocity   = config["rotate_velocity"].as<double>();
            cfg.rotate_velocity_x = config["rotate_velocity_x"].as<double>();
            cfg.rotate_velocity_y = config["rotate_velocity_y"].as<double>();

            cfg.pivot_ball_velocity   = config["pivot_ball_velocity"].as<double>();
            cfg.pivot_ball_velocity_x = config["pivot_ball_velocity_x"].as<double>();
            cfg.pivot_ball_velocity_y = config["pivot_ball_velocity_y"].as<double>();

            cfg.target_strength   = config["target_strength"].as<double>();
            cfg.heading_strength  = config["heading_strength"].as<double>();
            cfg.obstacle_strength = config["obstacle_strength"].as<double>();
            cfg.bounds_strength   = config["bounds_strength"].as<double>();

            // Emit empty Robots in case we don't get any
            emit(std::make_unique<Robots>());
        });

        // Path to walk to a particular point
        on<Provide<WalkTo>, Uses<Walk>, With<Robots>, With<Field>, With<Sensors>>().then([this](
                                                                                             const WalkTo& walk_to,
                                                                                             const Uses<Walk>& walk,
                                                                                             const Robots& robots,
                                                                                             const Field& field,
                                                                                             const Sensors& sensors) {
            // If we haven't got an active walk task, then reset the velocity to minimum velocity
            if (walk.run_state == GroupInfo::RunState::NO_TASK) {
                velocity_magnitude = cfg.min_translational_velocity_magnitude;
            }

            // If robot getting close to the point, begin to decelerate to minimum velocity
            if (walk_to.rPRr.head(2).norm() < cfg.approach_radius) {
                velocity_magnitude -= cfg.acceleration;
                velocity_magnitude = std::max(velocity_magnitude, cfg.min_translational_velocity_magnitude);
            }
            else {
                // If robot is far away from the point, accelerate to max velocity
                velocity_magnitude += cfg.acceleration;
                velocity_magnitude = std::max(cfg.min_translational_velocity_magnitude,
                                              std::min(velocity_magnitude, cfg.max_translational_velocity_magnitude));
            }

            // Obtain the unit vector to desired target in robot space and scale by cfg.translational_velocity
            // Eigen::Vector3d velocity_target = walk_to.rPRr.normalized() * velocity_magnitude;
            Eigen::Isometry3d Hfr = field.Hfw * sensors.Hrw.inverse();
            Eigen::Vector2d rPFf  = (Hfr * walk_to.rPRr).head(2);
            Eigen::Vector2d rRFf  = Hfr.translation().head(2);

            std::vector<Eigen::Vector2d> obstacles{};
            for (const auto& robot : robots.robots) {
                obstacles.push_back((field.Hfw * robot.rRWw).head(2));
            }

            // Get the vector field
            Eigen::Vector2d velocity_target = vector_field(rRFf, rPFf, walk_to.heading, obstacles);
            velocity_target =
                (Hfr.inverse().rotation() * Eigen::Vector3d(velocity_target.x(), velocity_target.y(), 0)).head(2);
            velocity_target = velocity_target.normalized() * velocity_magnitude;
            log<NUClear::INFO>("Vector Field", velocity_target.transpose(), velocity_magnitude, robots.robots.size());
            // Set the angular velocity component of the velocity_target with the angular displacement and saturate with
            // value cfg.max_angular_velocity
            double h = utility::math::clamp(cfg.min_angular_velocity, walk_to.heading, cfg.max_angular_velocity);

            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d(velocity_target.x(), velocity_target.y(), h)));
        });

        on<Provide<TurnOnSpot>>().then([this](const TurnOnSpot& turn_on_spot) {
            // Determine the direction of rotation
            int sign = turn_on_spot.clockwise ? -1 : 1;

            // Turn on the spot
            emit<Task>(std::make_unique<Walk>(
                Eigen::Vector3d(cfg.rotate_velocity_x, cfg.rotate_velocity_y, sign * cfg.rotate_velocity)));
        });

        on<Provide<TurnAroundBall>>().then([this](const TurnAroundBall& turn_around_ball) {
            // Determine the direction of rotation
            int sign = turn_around_ball.clockwise ? -1 : 1;
            // Turn around the ball
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d(cfg.pivot_ball_velocity_x,
                                                              sign * cfg.pivot_ball_velocity_y,
                                                              sign * cfg.pivot_ball_velocity)));
        });
    }
}  // namespace module::planning
