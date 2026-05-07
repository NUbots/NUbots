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

    // TODO: Figure out whether backwards step should be the primary behaviour for AvoidRobot
    static Eigen::Vector2d backwards_step(const Eigen::Vector2d& retreat_vec,
                                          const Eigen::Vector2d& current_blend,
                                          double min_valid_obstacle_distance) {
        if (current_blend.squaredNorm() <= min_valid_obstacle_distance) {
            return retreat_vec;  // fallback pure retreat
        }
        Eigen::Vector2d normalized = current_blend;
        normalized.normalize();
        return normalized;
    }

    AvoidRobot::AvoidRobot(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Startup>().then([this] { log<DEBUG>("AvoidRobot loaded"); });

        on<Configuration>("AvoidRobot.yaml").then([this](const Configuration& config) {
            // Use configuration here from file AvoidRobot.yaml
            this->log_level                   = config["log_level"].as<NUClear::LogLevel>();
            cfg.min_distance_threshold        = config["min_distance_threshold"].as<double>();
            cfg.threshold_margin              = config["threshold_margin"].as<double>();
            cfg.avoidance_walk_speed          = config["avoidance_walk_speed"].as<double>();
            cfg.min_valid_obstacle_distance   = config["min_valid_obstacle_distance"].as<double>();
            cfg.lateral_avoidance_weight      = config["lateral_avoidance_weight"].as<double>();
            cfg.retreat_avoidance_weight      = config["retreat_avoidance_weight"].as<double>();
            cfg.min_forward_obstacle_x        = config["min_forward_obstacle_x"].as<double>();
            cfg.near_field_avoidance_distance = config["near_field_avoidance_distance"].as<double>();
        });

        on<Provide<AvoidRobotTask>, With<Robots>, With<Sensors>>().then([this](
                                                                            const std::shared_ptr<const Robots>& robots,
                                                                            const Sensors& sensors) {
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

                // Use a deterministic sidestep direction so we do not oscillate left-right each tick
                const Eigen::Vector2d left_perpendicular(-nearest_obstacle.y(), nearest_obstacle.x());
                const Eigen::Vector2d right_perpendicular(nearest_obstacle.y(), -nearest_obstacle.x());
                const Eigen::Vector2d side_direction =
                    nearest_obstacle.y() >= 0.0 ? right_perpendicular.normalized() : left_perpendicular.normalized();

                // Failsafe behaviour to mostly sidestep to leave the collision line, with slight retreat
                Eigen::Vector2d blended_direction =
                    (cfg.lateral_avoidance_weight * side_direction) + (cfg.retreat_avoidance_weight * away_direction);

                // If blend weights collapse to an invalid tiny vector, fall back to a stable pure retreat vector
                blended_direction = backwards_step(away_direction, blended_direction, cfg.min_valid_obstacle_distance);

                const Eigen::Vector3d velocity_target(cfg.avoidance_walk_speed * blended_direction.x(),
                                                      cfg.avoidance_walk_speed * blended_direction.y(),
                                                      0.0);

                emit<Task>(std::make_unique<WalkProposal>(velocity_target));
                log<INFO>(
                    fmt::format("Avoiding nearest robot at ({:.3f}, {:.3f}) distance {:.3f}m with velocity ({:.3f}, "
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

}  // namespace module::strategy
