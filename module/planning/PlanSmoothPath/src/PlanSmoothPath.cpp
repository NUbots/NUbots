/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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
#include "PlanSmoothPath.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/planning/WalkPath.hpp"
#include "message/skill/Walk.hpp"

namespace module::planning {

    using extension::Configuration;
    using message::planning::WalkProposal;
    using message::skill::Walk;

    PlanSmoothPath::PlanSmoothPath(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanSmoothPath.yaml").then([this](const Configuration& config) {
            this->log_level                  = config["log_level"].as<NUClear::LogLevel>();
            cfg.max_acceleration             = config["max_acceleration"].as<double>();
            cfg.max_translational_velocity_x = config["max_translational_velocity_x"].as<double>();
            cfg.max_translational_velocity_y = config["max_translational_velocity_y"].as<double>();
            cfg.max_angular_velocity         = config["max_angular_velocity"].as<double>();
        });

        // Intercept Walk commands and apply smoothing
        on<Provide<WalkProposal>>().then([this](const WalkProposal& walk) {
            // Apply acceleration limiting and velocity constraints
            Eigen::Vector3d smoothed_command = apply_acceleration_limiting(walk.velocity_target);
            smoothed_command                 = constrain_velocity(smoothed_command);

            // Create new smoothed walk command
            auto smoothed_walk = std::make_unique<Walk>(smoothed_command);

            // Store for next iteration
            previous_walk_command = smoothed_command;
            last_update_time      = NUClear::clock::now();

            // Forward the smoothed command to the actual Walk skill
            emit<Task>(std::move(smoothed_walk));
        });
    }

    Eigen::Vector3d PlanSmoothPath::apply_acceleration_limiting(const Eigen::Vector3d& target_command) {
        auto current_time = NUClear::clock::now();
        double dt         = std::chrono::duration<double>(current_time - last_update_time).count();

        // Calculate maximum allowed change in velocity components
        double max_delta = cfg.max_acceleration * dt;

        Eigen::Vector3d limited_command = target_command;

        // Limit each component's acceleration
        for (int i = 0; i < 3; i++) {
            double delta = target_command[i] - previous_walk_command[i];
            if (std::abs(delta) > max_delta) {
                limited_command[i] = previous_walk_command[i] + (delta > 0 ? max_delta : -max_delta);
            }
        }

        return limited_command;
    }

    Eigen::Vector3d PlanSmoothPath::constrain_velocity(const Eigen::Vector3d& velocity_command) {
        Eigen::Vector2d translational_velocity = velocity_command.head<2>();

        // If either translational component exceeds the limit, scale the vector to fit within the limits
        if (std::abs(velocity_command.x()) >= cfg.max_translational_velocity_x
            || std::abs(velocity_command.y()) >= cfg.max_translational_velocity_y) {
            double sx =
                velocity_command.x() != 0.0 ? cfg.max_translational_velocity_x / std::abs(velocity_command.x()) : 0.0;
            double sy =
                velocity_command.y() != 0.0 ? cfg.max_translational_velocity_y / std::abs(velocity_command.y()) : 0.0;
            // Select the minimum scale factor to ensure neither limit is exceeded but direction is maintained
            double s               = std::min(sx, sy);
            translational_velocity = velocity_command.head<2>() * s;
        }

        // Ensure the angular velocity is within the limits
        double angular_velocity = std::clamp(velocity_command.z(), -cfg.max_angular_velocity, cfg.max_angular_velocity);

        return Eigen::Vector3d(translational_velocity.x(), translational_velocity.y(), angular_velocity);
    }

}  // namespace module::planning
