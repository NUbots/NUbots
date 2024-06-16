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

#include "message/localisation/Ball.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/skill/Kick.hpp"
#include "message/skill/Walk.hpp"

#include "utility/math/comparison.hpp"

namespace module::planning {

    using extension::Configuration;

    using message::localisation::Ball;
    using message::planning::TurnAroundBall;
    using message::planning::TurnOnSpot;
    using message::planning::WalkDirect;
    using message::planning::WalkTo;
    using message::skill::Walk;

    PlanWalkPath::PlanWalkPath(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanWalkPath.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanWalkPath.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.enter_rotate_to_target  = config["enter_rotate_to_target"].as<double>();
            cfg.exit_rotate_to_target   = config["exit_rotate_to_target"].as<double>();
            cfg.enter_walk_to_target    = config["enter_walk_to_target"].as<double>();
            cfg.exit_walk_to_target     = config["exit_walk_to_target"].as<double>();
            cfg.enter_rotate_to_heading = config["enter_rotate_to_heading"].as<double>();
            cfg.exit_rotate_to_heading  = config["exit_rotate_to_heading"].as<double>();

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

            // Initial state for the planner
            emit(std::make_unique<State>(State::STOP));
        });

        on<Start<WalkTo>>().then([this] {
            // Reset the velocity magnitude to minimum velocity
            velocity_magnitude = cfg.min_translational_velocity_magnitude;

            // Initial state for the planner
            emit(std::make_unique<State>(State::STOP));
        });


        on<Provide<WalkTo>, When<State, std::equal_to, State::ROTATE_TO_TARGET>>().then([this](const WalkTo& walk_to) {
            // If the robot is close enough to the target, do not rotate to face the target
            if (walk_to.rPRr.head(2).norm() < cfg.enter_walk_to_target) {
                emit(std::make_unique<State>(State::ROTATE_TO_HEADING));
                return;
            }

            // If robot is facing the target, transition to the next state
            double angle = std::atan2(walk_to.rPRr.y(), walk_to.rPRr.x());
            if (std::abs(angle) < cfg.exit_rotate_to_target) {
                emit(std::make_unique<State>(State::WALK_TO_TARGET));
            }
            else {
                // Rotate to face the target
                emit<Task>(std::make_unique<TurnOnSpot>(angle < 0.0));
            }
        });

        on<Provide<WalkTo>, When<State, std::equal_to, State::WALK_TO_TARGET>>().then([this](const WalkTo& walk_to) {
            // If the robot is close enough to the target, do not walk to the target
            if (walk_to.rPRr.head(2).norm() < cfg.exit_walk_to_target) {
                emit(std::make_unique<State>(State::ROTATE_TO_HEADING));
                return;
            }

            // If the robot isn't within threshold to face target, rotate to face target
            // Enter threshold is larger than exit, so it is harder to reenter rotating to the target
            if (std::abs(std::atan2(walk_to.rPRr.y(), walk_to.rPRr.x())) > cfg.enter_rotate_to_target) {
                emit(std::make_unique<State>(State::ROTATE_TO_TARGET));
                return;
            }

            emit<Task>(std::make_unique<WalkDirect>(walk_to.rPRr, walk_to.heading));
        });

        on<Provide<WalkTo>, When<State, std::equal_to, State::ROTATE_TO_HEADING>>().then([this](const WalkTo& walk_to) {
            // If the robot is facing the target heading and close to the target, stop
            double distance = walk_to.rPRr.head(2).norm();
            if (std::abs(walk_to.heading) < cfg.exit_rotate_to_heading && distance < cfg.enter_walk_to_target) {
                emit(std::make_unique<State>(State::STOP));
            }

            // If the robot is too far out of the position, walk to position
            if (distance > cfg.enter_walk_to_target) {
                emit(std::make_unique<State>(State::WALK_TO_TARGET));
                return;
            }

            // Robot is not facing the heading but is close enough, rotate to face the heading
            emit<Task>(std::make_unique<TurnOnSpot>(walk_to.heading < 0.0));
        });

        on<Provide<WalkTo>, When<State, std::equal_to, State::STOP>>().then([this](const WalkTo& walk_to) {
            // Check if the robot is far away from the target and not facing the target, then face target
            double angle = std::atan2(walk_to.rPRr.y(), walk_to.rPRr.x());
            if (walk_to.rPRr.head(2).norm() > cfg.enter_walk_to_target
                && std::abs(angle) > cfg.enter_rotate_to_target) {
                emit(std::make_unique<State>(State::ROTATE_TO_TARGET));
                return;
            }

            // Check if the robot is far from the target
            if (walk_to.rPRr.head(2).norm() > cfg.enter_walk_to_target) {
                emit(std::make_unique<State>(State::WALK_TO_TARGET));
                return;
            }

            // The robot is close to the target
            // Check if the robot is facing the target heading
            if (std::abs(walk_to.heading) > cfg.enter_rotate_to_heading) {
                emit(std::make_unique<State>(State::ROTATE_TO_HEADING));
                return;
            }

            // The robot is close to the target and facing the target heading, keep stopped
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()));
        });

        on<Provide<WalkDirect>>().then([this](const WalkDirect& walk_direct) {
            // If robot getting close to the point, begin to decelerate to minimum velocity
            if (walk_direct.rPRr.head(2).norm() < cfg.approach_radius) {
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
            Eigen::Vector3d velocity_target = walk_direct.rPRr.normalized() * velocity_magnitude;

            // Set the angular velocity component of the velocity_target with the angular displacement and saturate with
            // value cfg.max_angular_velocity
            velocity_target.z() =
                utility::math::clamp(cfg.min_angular_velocity, walk_direct.heading, cfg.max_angular_velocity);

            emit<Task>(std::make_unique<Walk>(velocity_target));
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
