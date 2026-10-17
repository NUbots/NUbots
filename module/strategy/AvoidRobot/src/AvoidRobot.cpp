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
#include "AvoidRobot.hpp"

#include <fmt/format.h>
#include <ranges>

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Robot.hpp"
#include "message/strategy/AvoidRobot.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Robots;
    using message::planning::WalkProposal;
    using AvoidRobotTask = message::strategy::AvoidRobot;

    AvoidRobot::AvoidRobot(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Startup>().then([this] { log<DEBUG>("AvoidRobot loaded"); });

        on<Configuration>("AvoidRobot.yaml").then([this](const Configuration& config) {
            // Use configuration here from file AvoidRobot.yaml
            this->log_level                   = config["log_level"].as<NUClear::LogLevel>();
            cfg.min_distance_threshold        = config["min_distance_threshold"].as<double>();
            cfg.threshold_margin              = config["threshold_margin"].as<double>();
            cfg.max_x_avoidance_velocity      = config["max_x_avoidance_velocity"].as<double>();
            cfg.max_y_avoidance_velocity      = config["max_y_avoidance_velocity"].as<double>();
            cfg.min_valid_obstacle_distance   = config["min_valid_obstacle_distance"].as<double>();
            cfg.lateral_avoidance_weight      = config["lateral_avoidance_weight"].as<double>();
            cfg.retreat_avoidance_weight      = config["retreat_avoidance_weight"].as<double>();
            cfg.min_forward_obstacle_x        = config["min_forward_obstacle_x"].as<double>();
            cfg.near_field_avoidance_distance = config["near_field_avoidance_distance"].as<double>();
        });

        // Provide WalkProposal so Director can choose this provider when a WalkProposal is requested.
        // Use Optional<With<Robots>> since some call sites may not include robot observations.
        on<Provide<AvoidRobotTask>, With<Robots>, Trigger<Sensors>>().then(
            [this](const Robots& robots, const Sensors& sensors) {
            const auto& Hrw = sensors.Hrw;

            if (robots.robots.empty()) {
                avoid_active = false;
                log<DEBUG>("AvoidRobot tick: no robots available");
                return;
            }

            std::vector<Eigen::Vector2d> all_obstacles{};
            all_obstacles.reserve(robots.robots.size());
            for (const auto& robot : robots.robots) {
                all_obstacles.emplace_back((Hrw * robot.rRWw).head(2));
            }

            std::ranges::sort(all_obstacles, {}, &Eigen::Vector2d::squaredNorm);

            const Eigen::Vector2d nearest_obstacle = all_obstacles.front();
            const double nearest_dist_to_opp       = nearest_obstacle.norm();
            // Front-gate to normally only avoid obstacles that are ahead in robot frame
            const bool obstacle_in_front = nearest_obstacle.x() > cfg.min_forward_obstacle_x;
            // Near-field override to force avoidance only at very close range even if obstacle is side/behind
            const bool near_field_obstacle = nearest_dist_to_opp < cfg.near_field_avoidance_distance;

            // Enter avoidance when either (a) close enough to be immediately unsafe, or (b) in-front and within
            // the standard activation threshold
            const bool should_enter_avoidance =
                near_field_obstacle ? true : (obstacle_in_front && nearest_dist_to_opp < cfg.min_distance_threshold);

            // Exit avoidance only once we are no longer in the forced near-field zone and either not in-front
            // anymore or far enough away that we avoid rapid on/off switching near the distance threshold.
            const bool should_exit_avoidance =
                (!near_field_obstacle)
                && (!obstacle_in_front || nearest_dist_to_opp > (cfg.min_distance_threshold + cfg.threshold_margin));

            if (should_enter_avoidance) {
                avoid_active = true;
                log<DEBUG>("Setting Avoidance mode to TRUE");
            }
            else if (should_exit_avoidance) {
                avoid_active = false;
                log<DEBUG>("Setting Avoidance mode to FALSE");
            }

            if (avoid_active && nearest_dist_to_opp > cfg.min_valid_obstacle_distance) {
                const Eigen::Vector2d away_direction = -nearest_obstacle.normalized();

                // Construct velocity in robot frame and constrain to avoidance-specific maxima
                Eigen::Vector3d velocity_target(cfg.max_x_avoidance_velocity * away_direction.x(),
                                                cfg.max_y_avoidance_velocity * away_direction.y(),
                                                0.0);

                velocity_target = constrain_velocity(velocity_target);

                emit<Task>(std::make_unique<WalkProposal>(velocity_target));
                log<DEBUG>(fmt::format(
                    "Avoiding nearest robot at ({:.3f}, {:.3f}) distance {:.3f}m with velocity ({:.3f}, "
                    "{:.3f}, {:.3f})",
                    nearest_obstacle.x(),
                    nearest_obstacle.y(),
                    nearest_dist_to_opp,
                    velocity_target.x(),
                    velocity_target.y(),
                    velocity_target.z()));
            }
        });
    }

    //TODO: Put commment about this function
    Eigen::Vector3d AvoidRobot::constrain_velocity(const Eigen::Vector3d& v) {
        // Scale factors in each translational axis (∞ if the component is 0)
        const auto inf = std::numeric_limits<double>::infinity();
        const auto sx  = v.x() ? cfg.max_x_avoidance_velocity / std::abs(v.x()) : inf;
        const auto sy  = v.y() ? cfg.max_y_avoidance_velocity / std::abs(v.y()) : inf;

        // no scaling (s=1) unless either axis exceeds the limit
        const auto s = std::min({1.0, sx, sy});

        // Angular component is not used for avoidance proposals in current config; pass-through
        return {v.x() * s, v.y() * s, v.z()};
    }

}  // namespace module::strategy
