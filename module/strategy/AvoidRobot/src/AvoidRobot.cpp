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

#include <ranges>

#include <fmt/format.h>

#include "extension/Configuration.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Robot.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::localisation::Robots;
    using message::planning::WalkProposal;
    using message::planning::WalkTo;
    using message::input::Sensors;

    AvoidRobot::AvoidRobot(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Startup>().then([this] { log<INFO>("AvoidRobot loaded"); });

        on<Configuration>("AvoidRobot.yaml").then([this](const Configuration& config) {
            // Use configuration here from file AvoidRobot.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.min_distance_threshold = config["min_distance_threshold"].as<double>();
            cfg.threshold_margin = config["threshold_margin"].as<double>();
            cfg.avoidance_walk_speed = config["avoidance_walk_speed"].as<double>();
            cfg.min_valid_obstacle_distance = config["min_valid_obstacle_distance"].as<double>();
        });

        on<Provide<WalkTo>>().then([this](const WalkTo& walk_to) {
            // Storing the latest walk_to message received along with a timestamp
            latest_walk_to = walk_to;
            latest_walk_to_time = NUClear::clock::now();
            log<INFO>("Cached latest walk_to", latest_walk_to_time);
        });

        on<Provide<WalkTo>, Optional<With<Robots>>, With<Sensors>>().then(
            [this](const WalkTo& walk_to, const std::shared_ptr<const Robots>& robots, const Sensors& sensors) {

            const auto& Hrw = sensors.Hrw;
            if (!robots || robots->robots.empty()) {
                avoid_active = false;
                log<DEBUG>("AvoidRobot tick: no robots available");
                return;
            }

            std::vector<Eigen::Vector2d> all_obstacles{};
            all_obstacles.reserve(robots->robots.size());
            for (const auto& robot : robots->robots) {
                all_obstacles.emplace_back((Hrw * robot.rRWw).head(2));
            }

            std::ranges::sort(all_obstacles, {}, &Eigen::Vector2d::squaredNorm);

            const Eigen::Vector2d nearest_obstacle = all_obstacles.front();
            const double nearest_dist_to_opp       = nearest_obstacle.norm();

            if (nearest_dist_to_opp < cfg.min_distance_threshold) {
                avoid_active = true;
            }
            else if (nearest_dist_to_opp > (cfg.min_distance_threshold + cfg.threshold_margin)) {
                avoid_active = false;
            }

            if (avoid_active && nearest_dist_to_opp > cfg.min_valid_obstacle_distance) {
                //TODO: Temporary direction, needs better path planning
                const Eigen::Vector2d away_direction = -nearest_obstacle.normalized();
                const Eigen::Vector3d velocity_target(cfg.avoidance_walk_speed * away_direction.x(),
                                                      cfg.avoidance_walk_speed * away_direction.y(),
                                                      0.0);

                emit<Task>(std::make_unique<WalkProposal>(velocity_target));
                log<DEBUG>(fmt::format("Avoiding robot at ({:.3f}, {:.3f}) distance {:.3f}m with velocity ({:.3f}, {:.3f}, {:.3f})",
                                       nearest_obstacle.x(),
                                       nearest_obstacle.y(),
                                       nearest_dist_to_opp,
                                       velocity_target.x(),
                                       velocity_target.y(),
                                       velocity_target.z()));
            }

        });

    }

}  // namespace module::strategy
