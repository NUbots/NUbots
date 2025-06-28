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

#ifndef UTILITY_BEHAVIOUR_OBSTACLE_DETECTION_HPP
#define UTILITY_BEHAVIOUR_OBSTACLE_DETECTION_HPP

#include <Eigen/Core>
#include <vector>
#include <algorithm>
#include <utility>
#include <cmath>
#include <nuclear>

#include "utility/math/geometry/intersection.hpp"

namespace utility::behaviour {
    using utility::math::geometry::intersection_line_and_circle;

/**
 * @brief Finds the closest obstacle intersecting the path and groups nearby obstacles.
 *
 * @param all_obstacles All obstacles in the world (positions relative to robot).
 * @param rDRr Vector from robot to target.
 * @param obstacle_radius The radius to use for obstacle avoidance.
 * @param debug If true, print debug logs.
 * @return Pair: (bool behind_target, vector of grouped obstacles)
 */
inline std::pair<bool, std::vector<Eigen::Vector2d>> get_obstacles(
    const std::vector<Eigen::Vector2d>& all_obstacles,
    const Eigen::Vector2d& rDRr,
    double obstacle_radius,
    bool debug = false) {
    // Find the closest obstacle that intersects the path
    double min_dist = std::numeric_limits<double>::max();
    std::optional<Eigen::Vector2d> closest_obstacle;
    for (const auto& obstacle : all_obstacles) {
        const bool in_front = rDRr.normalized().dot(obstacle.normalized()) > 0.0;
        const bool intersects = intersection_line_and_circle(Eigen::Vector2d::Zero(), rDRr, obstacle, obstacle_radius);
        double dist = obstacle.norm();
        if (debug) {
            NUClear::log<NUClear::LogLevel::DEBUG>("Obstacle check:", obstacle.transpose(), "in_front:", in_front, "intersects:", intersects, "dist:", dist);
        }
        if (in_front && intersects && dist < min_dist) {
            min_dist = dist;
            closest_obstacle = obstacle;
        }
    }
    std::vector<Eigen::Vector2d> grouped_obstacles;
    bool behind_target = false;
    if (closest_obstacle) {
        // Group close obstacles for clearance
        for (const auto& obstacle : all_obstacles) {
            if ((obstacle - *closest_obstacle).norm() < obstacle_radius * 3) {
                grouped_obstacles.push_back(obstacle);
            }
        }
        // Determine if the closest obstacle is behind the target
        behind_target = (closest_obstacle->norm() >= rDRr.norm());
        if (debug) {
            NUClear::log<NUClear::LogLevel::DEBUG>("Closest obstacle:", closest_obstacle->transpose(), "behind_target:", behind_target);
        }
    }
    return {behind_target, grouped_obstacles};
}

} // namespace utility::behaviour

#endif  // UTILITY_BEHAVIOUR_OBSTACLE_DETECTION_HPP
