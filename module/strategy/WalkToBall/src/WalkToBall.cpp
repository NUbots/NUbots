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
#include "WalkToBall.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/Robot.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/euler.hpp"
#include "utility/math/geometry/intersection.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::strategy {

    using extension::Configuration;

    using WalkToBallTask   = message::strategy::WalkToBall;
    using FieldDescription = message::support::FieldDescription;

    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::localisation::Robots;
    using message::planning::WalkTo;
    using message::strategy::PositionBehindBall;
    using message::strategy::TackleBall;
    using message::strategy::WalkToFieldPosition;
    using message::strategy::WalkToKickBall;

    using utility::math::angle::normalise_angle;
    using utility::math::angle::vector_to_bearing;
    using utility::math::euler::pos_rpy_to_transform;
    using utility::math::geometry::intersection_line_and_circle;

    using utility::support::Expression;

    WalkToBall::WalkToBall(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("WalkToBall.yaml").then([this](const Configuration& config) {
            // Use configuration here from file WalkToBall.yaml
            this->log_level            = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_y_offset          = config["ball_y_offset"].as<double>();
            cfg.ball_kick_distance     = config["ball_kick_distance"].as<double>();
            cfg.ball_approach_distance = config["ball_approach_distance"].as<double>();
            cfg.goal_target_offset     = config["goal_target_offset"].as<double>();
            cfg.max_angle_error        = config["max_angle_error"].as<Expression>();
            cfg.avoid_ball_offset      = Eigen::Vector3d(config["avoid_ball_offset"].as<Expression>());
            cfg.avoid_opponent_offset  = config["avoid_opponent_offset"].as<double>();
            cfg.approach_offset        = config["approach_offset"].as<double>();
            cfg.tackle_angle_offset    = config["tackle_angle_offset"].as<double>();
            cfg.distance_behind_ball   = config["distance_behind_ball"].as<double>();
            cfg.max_error_y            = config["max_error_y"].as<double>();
            cfg.error_gain_perp        = config["error_gain_perp"].as<double>();
            cfg.min_offset_y           = config["min_offset_y"].as<double>();
            cfg.max_offset_y           = config["max_offset_y"].as<double>();
            cfg.infront_of_ball_radius = config["infront_of_ball_radius"].as<double>();
            cfg.infront_check_distance = config["infront_check_distance"].as<double>();
            cfg.obstacle_radius        = config["obstacle_radius"].as<double>();
            cfg.goal_width_margin      = config["goal_width_margin"].as<double>();
        });

        on<Startup, Trigger<FieldDescription>>().then("Update Goal Position", [this](const FieldDescription& fd) {
            // Update the goal position
            rGFf = Eigen::Vector3d(-fd.dimensions.field_length / 2 - cfg.goal_target_offset, 0, 0);
        });

        // If the Provider updates on Every and the last Ball was too long ago, it won't emit any Task
        // Otherwise it will emit a Task to walk to the ball
        on<Provide<WalkToBallTask>, With<Ball>, With<Sensors>>().then([this](const Ball& ball, const Sensors& sensors) {
            Eigen::Vector3d rBRr = sensors.Hrw * ball.rBWw;
            // Add an offset to account for walking with the foot in front of the ball
            rBRr.y() += cfg.ball_y_offset;
            const double heading = std::atan2(rBRr.y(), rBRr.x());
            auto Hrb             = pos_rpy_to_transform(rBRr, Eigen::Vector3d(0, 0, heading));
            emit<Task>(std::make_unique<WalkTo>(Hrb));
        });

        // If the Provider updates on Every and the last Ball was too long ago, it won't emit any Task
        // Otherwise it will emit a Task to walk to the ball
        on<Provide<WalkToKickBall>,
           Optional<With<Robots>>,
           With<Ball>,
           With<Sensors>,
           With<Field>,
           With<FieldDescription>>()
            .then([this](const std::shared_ptr<const Robots>& robots,
                         const Ball& ball,
                         const Sensors& sensors,
                         const Field& field,
                         const FieldDescription& field_description) {
                // Ball position relative to robot in robot frame (rBRr)
                Eigen::Vector3d rBRr = sensors.Hrw * ball.rBWw;
                rBRr.y() += cfg.ball_y_offset;  // Offset for ball-walking alignment

                // Ball and goal positions in field frame
                const Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;
                const Eigen::Vector3d rGBf = rGFf - rBFf;        // Vector from ball to goal
                const Eigen::Vector3d uGBf = rGBf.normalized();  // Unit vector toward goal

                // Robot position and orientation in field frame
                Eigen::Isometry3d Hfr = field.Hfw * sensors.Hrw.inverse();
                Eigen::Vector3d rRFf  = Hfr.translation();  // Robot position in field

                // Compute desired heading angle (towards goal from ball)
                double desired_heading  = std::atan2(rGBf.y(), rGBf.x());
                Eigen::Vector3d robot_x = Hfr.linear().col(0);  // Robot forward direction in field
                double robot_heading    = std::atan2(robot_x.y(), robot_x.x());

                // Compute kick target position (directly behind the ball)
                Eigen::Vector3d target = rBFf - uGBf * cfg.ball_kick_distance;

                // Final walking target pose
                Eigen::Isometry3d Hfk = Eigen::Isometry3d::Identity();

                // Simplify below calculations
                auto progress     = [](auto val, auto max_val) { return std::clamp(val / max_val, 0.0, 1.0); };
                const auto uGBf_p = Eigen::Vector3d(-uGBf.y(), uGBf.x(), 0);  // perpendicular

                // Error in the direction parallel to the ball-goal vector
                const double err_x = (rRFf - rBFf).dot(uGBf);
                // Error in the direction perpendicular to the ball-goal vector
                const double err_y = (rRFf - rBFf).dot(uGBf_p);
                // Heading error (difference between robot and desired heading)
                const double err_z = normalise_angle(desired_heading - robot_heading);

                /* Adjustment target point for a better ball approach */

                // Add a sideways offset if we're in front of the ball (so we walk around it)
                if (err_x > 0) {
                    target += uGBf_p * (err_y < 0 ? -1 : 1) * std::clamp(err_x, cfg.min_offset_y, cfg.max_offset_y);
                }

                // Aim for further behind the ball if we are perpendicular misaligned
                target -= uGBf * progress(std::abs(err_y), cfg.max_error_y) * cfg.error_gain_perp;

                // Aim for further behind the ball if our heading angle is wrong (to give us time to align)
                target -= uGBf * cfg.ball_approach_distance * progress(std::abs(err_z), cfg.max_angle_error);

                // If there are robots, check if there are obstacles in the way
                if (robots) {
                    // Get the positions of all robots in the world
                    std::vector<Eigen::Vector3d> all_obstacles{};
                    for (const auto& robot : robots->robots) {
                        all_obstacles.emplace_back(field.Hfw * robot.rRWw);
                    }
                    // Sort obstacles based on distance from the robot
                    std::ranges::sort(all_obstacles, {}, [&Hfr](const auto& obstacle) {
                        return (Hfr.inverse() * obstacle).squaredNorm();
                    });

                    auto obstacle = robot_infront_of_path(all_obstacles, rBFf, rGFf);

                    if (obstacle.has_value()) {
                        log<DEBUG>("Avoiding obstacle in the way of kick path");

                        // Define goal boundaries for side selection
                        const double right_boundary =
                            field_description.dimensions.goal_width / 2.0 - cfg.goal_width_margin;
                        const double left_boundary = -right_boundary;

                        // Calculate avoidance points and angles for both sides
                        const Eigen::Vector3d perp(rGBf.normalized().y(), -rGBf.normalized().x(), 0);
                        auto calculate_avoidance = [&](int sign) {
                            const Eigen::Vector3d avoid_point = *obstacle + perp * (sign * cfg.obstacle_radius);
                            const double angle                = vector_to_bearing(avoid_point.head(2) - rBFf.head(2));
                            const Eigen::Vector3d target =
                                rBFf - (avoid_point - rBFf).normalized() * cfg.ball_approach_distance;
                            return std::make_pair(target, angle);
                        };

                        const auto [left_target, left_angle]   = calculate_avoidance(-1);
                        const auto [right_target, right_angle] = calculate_avoidance(1);

                        // Determine which side to go around based on field position and robot heading
                        const bool go_right =
                            (left_boundary <= obstacle->y() && obstacle->y() <= right_boundary)
                                ? (robot_heading > 0)  // Middle of field: choose based on robot heading direction
                                : (obstacle->y() < left_boundary);  // Side of field: go to the opposite side

                        // Adjust target and headings based on side selection
                        target          = go_right ? right_target : left_target;
                        desired_heading = go_right ? right_angle : left_angle;

                        // If aligned, walk directly into the ball
                        const double angle_error = normalise_angle(desired_heading - robot_heading);
                        if (std::abs(angle_error) < (cfg.max_angle_error / 2)) {
                            log<DEBUG>("Facing the right direction, walking into the ball");
                            target = rBFf - uGBf * cfg.ball_kick_distance;
                        }
                    }
                }

                Hfk = pos_rpy_to_transform(target, Eigen::Vector3d(0, 0, desired_heading));

                // Issue walking task toward final position and orientation
                emit<Task>(std::make_unique<WalkToFieldPosition>(Hfk, false));
            });


        // If the Provider updates on Every and the last Ball was too long ago, it won't emit any Task
        // Otherwise it will emit a Task to walk to the ball
        on<Provide<TackleBall>, With<Ball>, With<Sensors>, With<Field>, With<Robots>>().then(
            [this](const Ball& ball, const Sensors& sensors, const Field& field, const Robots& robots) {
                log<DEBUG>("Tackling the ball");
                // Position of the ball relative to the robot in the robot space
                Eigen::Vector3d rBRr = sensors.Hrw * ball.rBWw;
                // Add an offset to account for walking with the foot in front of the ball
                rBRr.y() += cfg.ball_y_offset;

                // Position of the ball relative to the field in the field space
                Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;

                // If there are no robots, we cannot tackle the ball
                if (robots.robots.size() == 0) {
                    return;
                }

                // Find the robot in possession
                Eigen::Vector3d rOBf = (field.Hfw * robots.robots[0].rRWw) - rBFf;
                // Loop over all opponent robots and find the one closest to the ball
                for (const auto& robot : robots.robots) {
                    if (!robot.teammate) {
                        // field.Hfw * robot.rRWw -> rRFf
                        Eigen::Vector3d candidate_rOBf = (field.Hfw * robot.rRWw) - rBFf;
                        if (candidate_rOBf.norm() < rOBf.norm()) {
                            rOBf = candidate_rOBf;
                        }
                    }
                }

                // Normalise the vector
                Eigen::Vector3d uOBf = rOBf.normalized();

                // Get position of robot on field
                Eigen::Isometry3d Hfr   = field.Hfw * sensors.Hrw.inverse();
                Eigen::Vector3d rRFf    = Hfr.translation();
                Eigen::Vector3d robot_x = Hfr.linear().col(0);  // Robot forward direction in field
                double robot_heading    = std::atan2(robot_x.y(), robot_x.x());

                // Create left/right perpendicular unit vectors to opponent direction
                Eigen::Vector3d uLeft  = Eigen::Vector3d(-uOBf.y(), uOBf.x(), 0);  // 90 deg left
                Eigen::Vector3d uRight = Eigen::Vector3d(uOBf.y(), -uOBf.x(), 0);  // 90 deg right

                // Offset positions from the ball on both sides
                Eigen::Vector3d rLeftFf  = rBFf + uLeft * cfg.approach_offset;
                Eigen::Vector3d rRightFf = rBFf + uRight * cfg.approach_offset;

                // Choose the side to approach based on which is closer to the robot
                // Unit vector of the heading in field space
                Eigen::Vector3d uHf = (rRFf - rLeftFf).norm() < (rRFf - rRightFf).norm() ? uRight : uLeft;
                double heading      = std::atan2(uHf.y(), uHf.x());
                // Error between the robot's heading and the desired heading perpendicular to the opponent
                double angle_error = std::atan2(std::sin(heading - robot_heading), std::cos(heading - robot_heading));

                // if not robot is not yet aligned, add the offset to approach to the side of the opponent
                Eigen::Vector3d rApproachFf = rBFf;
                if (std::abs(angle_error) > cfg.tackle_angle_offset) {
                    // Add y offset in the direction of the perpendicular vector
                    rApproachFf -= uHf * cfg.approach_offset;
                }
                // Always add some distance backwards from opponent along opponent vector
                rApproachFf -= uOBf * cfg.avoid_opponent_offset;

                Eigen::Isometry3d Hfk = pos_rpy_to_transform(rApproachFf, Eigen::Vector3d(0, 0, heading));
                emit<Task>(std::make_unique<WalkToFieldPosition>(Hfk, false));
            });

        // Walk and stop behind the ball, facing the goal. Avoids walking into the ball.
        on<Provide<PositionBehindBall>, With<Ball>, With<Sensors>, With<Field>>().then(
            [this](const Ball& ball, const Sensors& sensors, const Field& field) {
                log<DEBUG>("Positioning behind the ball");
                // Ball position relative to robot in robot frame (rBRr)
                Eigen::Vector3d rBRr = sensors.Hrw * ball.rBWw;
                rBRr.y() += cfg.ball_y_offset;  // Offset for ball-walking alignment

                // Ball and goal positions in field frame
                const Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;
                const Eigen::Vector3d rGBf = rGFf - rBFf;        // Vector from ball to goal
                const Eigen::Vector3d uGBf = rGBf.normalized();  // Unit vector toward goal

                // Robot position and orientation in field frame
                Eigen::Isometry3d Hfr = field.Hfw * sensors.Hrw.inverse();
                Eigen::Vector3d rRFf  = Hfr.translation();  // Robot position in field

                // Compute desired heading angle (towards goal from ball)
                double desired_heading  = std::atan2(rGBf.y(), rGBf.x());
                Eigen::Vector3d robot_x = Hfr.linear().col(0);  // Robot forward direction in field
                double robot_heading    = std::atan2(robot_x.y(), robot_x.x());

                // Heading error (difference between robot and desired heading)
                double angle_error =
                    std::atan2(std::sin(desired_heading - robot_heading), std::cos(desired_heading - robot_heading));

                // Compute kick target position (directly behind the ball)
                const Eigen::Vector3d kick_target = rBFf - uGBf * cfg.distance_behind_ball;

                // Final walking target pose
                Eigen::Isometry3d Hfk = Eigen::Isometry3d::Identity();

                // If robot is approaching from behind the ball (x-axis direction)
                if (rBFf.x() > rRFf.x()) {
                    // Compute left and right side positions to walk around the ball
                    Eigen::Vector3d rAlFf = rBFf + cfg.avoid_ball_offset;
                    Eigen::Vector3d rArFf = rBFf + cfg.avoid_ball_offset.cwiseProduct(Eigen::Vector3d(1, -1, 1));

                    // Choose the closer avoidance side
                    const Eigen::Vector3d& avoid_pos = (rRFf - rArFf).norm() < (rRFf - rAlFf).norm() ? rArFf : rAlFf;

                    // Walk to avoid point and face the final approach direction
                    Eigen::Vector3d approach = kick_target - uGBf * cfg.ball_approach_distance;
                    double heading           = std::atan2((approach - avoid_pos).y(), (approach - avoid_pos).x());
                    Hfk                      = pos_rpy_to_transform(avoid_pos, Eigen::Vector3d(0, 0, heading));
                }
                // Otherwise, walk directly to scaled approach position
                else {
                    double angle_scale     = std::clamp(std::abs(angle_error) / cfg.max_angle_error, 0.0, 1.0);
                    Eigen::Vector3d target = kick_target - uGBf * cfg.ball_approach_distance * angle_scale;
                    Hfk                    = pos_rpy_to_transform(target, Eigen::Vector3d(0, 0, desired_heading));
                }

                // Issue walking task toward final position and orientation
                // Stop when at the target position
                emit<Task>(std::make_unique<WalkToFieldPosition>(Hfk, true));
            });
    }

    std::optional<Eigen::Vector3d> WalkToBall::robot_infront_of_path(const std::vector<Eigen::Vector3d>& all_obstacles,
                                                                     const Eigen::Vector3d& rBFf,
                                                                     const Eigen::Vector3d& rGFf) {
        for (const auto& obstacle : all_obstacles) {
            const bool in_front     = obstacle.x() < rBFf.x();
            const bool within_range = (obstacle - rBFf).norm() < cfg.infront_check_distance;
            const bool intersects_path =
                intersection_line_and_circle(rGFf.head(2), rBFf.head(2), obstacle.head(2), cfg.infront_of_ball_radius);
            if (in_front && within_range && intersects_path) {
                return obstacle;
            }
        }
        return std::nullopt;
    }

}  // namespace module::strategy
