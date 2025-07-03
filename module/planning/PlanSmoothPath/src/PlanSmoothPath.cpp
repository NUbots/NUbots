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

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"


namespace module::planning {

    using extension::Configuration;
    using message::planning::WalkProposal;
    using message::skill::Walk;
    using utility::nusight::graph;
    using utility::support::Expression;

    PlanSmoothPath::PlanSmoothPath(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanSmoothPath.yaml").then([this](const Configuration& config) {
            this->log_level                  = config["log_level"].as<NUClear::LogLevel>();
            cfg.max_acceleration             = config["max_acceleration"].as<double>();
            cfg.max_translational_velocity_x = config["max_translational_velocity_x"].as<double>();
            cfg.max_translational_velocity_y = config["max_translational_velocity_y"].as<double>();
            cfg.max_angular_velocity         = config["max_angular_velocity"].as<double>();
            cfg.tau                          = config["tau"].as<Expression>();
            // Ensure safety for division by zero if tau is zero
            cfg.alpha = (cfg.tau.array() > 0.01)
                            .select((1.0 - (-1.0 / (UPDATE_FREQUENCY * cfg.tau.array())).exp()).matrix(),
                                    Eigen::Vector3d::Ones());
            cfg.one_minus_alpha = Eigen::Vector3d::Ones() - cfg.alpha;
            log<INFO>("Smoothing walk with time constant tau: (",
                      cfg.tau.x(),
                      ", ",
                      cfg.tau.y(),
                      ", ",
                      cfg.tau.z(),
                      ") corresponding to alpha: (",
                      cfg.alpha.x(),
                      ", ",
                      cfg.alpha.y(),
                      ", ",
                      cfg.alpha.z(),
                      ")");
        });

        // Intercept Walk commands and apply smoothing
        on<Provide<WalkProposal>, Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>>().then([this](const WalkProposal&
                                                                                                        walk) {
            // Visualise the walk path in NUsight
            emit(graph("Walk Proposal", walk.velocity_target.x(), walk.velocity_target.y(), walk.velocity_target.z()));

            // Apply exponential smoothing to the walk command
            Eigen::Vector3d smoothed_command =
                walk.velocity_target.cwiseProduct(cfg.alpha) + previous_walk_command.cwiseProduct(cfg.one_minus_alpha);

            // Visualise the walk path in NUsight
            emit(graph("Smoothed Walk Command", smoothed_command.x(), smoothed_command.y(), smoothed_command.z()));

            Eigen::Vector3d smooth_diff = smoothed_command - walk.velocity_target;
            // Log the smoothing difference
            emit(graph("Walk Smoothing Difference", smooth_diff.x(), smooth_diff.y(), smooth_diff.z()));

            // Create new smoothed walk command
            auto smoothed_walk = std::make_unique<Walk>(smoothed_command);

            // Store for next iteration
            previous_walk_command = smoothed_command;
            last_update_time      = NUClear::clock::now();

            // Forward the smoothed command to the actual Walk skill
            emit<Task>(std::move(smoothed_walk));
        });
    }

}  // namespace module::planning
