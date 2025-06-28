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
 * @brief Detects obstacles between the robot and the target.
 *
 * @param all_obstacles All obstacles in the world (positions relative to robot).
 * @param rDRr Vector from robot to target.
 * @param obstacle_radius The radius to use for obstacle avoidance.
 * @param debug If true, print debug logs.
 * @return Pair of vectors: first = obstacles in the way (closer), second = obstacles to pivot around (not closer).
 */
inline std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> get_obstacles(
    const std::vector<Eigen::Vector2d>& all_obstacles,
    const Eigen::Vector2d& rDRr,
    double obstacle_radius,
    bool debug = false) {
    std::vector<Eigen::Vector2d> avoid_obstacles{};
    std::vector<Eigen::Vector2d> pivot_obstacles{};
    for (const auto& obstacle : all_obstacles) {
        Eigen::Vector2d rORr = obstacle;
        double proj = rORr.dot(rDRr.normalized());
        bool between = (proj > 0) && (proj < rDRr.norm());
        const bool in_front = rDRr.normalized().dot(obstacle.normalized()) > 0.0;
        const bool closer = obstacle.norm() < rDRr.norm();
        const bool intersects = intersection_line_and_circle(Eigen::Vector2d::Zero(), rDRr, obstacle, obstacle_radius);
        if (debug) {
            NUClear::log<NUClear::LogLevel::DEBUG>("Obstacle check:", obstacle.transpose(), "proj:", proj, "between:", between, "in_front:", in_front, "closer:", closer, "intersects:", intersects);
        }
        if (in_front && intersects) {
            if (closer) {
                if (debug) NUClear::log<NUClear::LogLevel::DEBUG>("Adding to avoid_obstacles:", obstacle.transpose());
                avoid_obstacles.push_back(obstacle);
            } else {
                if (debug) NUClear::log<NUClear::LogLevel::DEBUG>("Adding to pivot_obstacles:", obstacle.transpose());
                pivot_obstacles.push_back(obstacle);
            }
        }
    }
    // Group close obstacles for both lists
    auto group_close = [&](std::vector<Eigen::Vector2d>& group) {
        for (const auto& obstacle : all_obstacles) {
            for (const auto& grouped : group) {
                NUClear::log<NUClear::LogLevel::DEBUG>("Obstacle radius", obstacle_radius);
                if ((obstacle - grouped).norm() < obstacle_radius * 3) {
                    if (std::find(group.begin(), group.end(), obstacle) == group.end()) {
                        group.push_back(obstacle);
                    }
                    break;
                }
            }
        }
    };
    if (!avoid_obstacles.empty()) group_close(avoid_obstacles);
    if (!pivot_obstacles.empty()) group_close(pivot_obstacles);
    return { avoid_obstacles, pivot_obstacles };
}

} // namespace utility::behaviour

#endif  // UTILITY_BEHAVIOUR_OBSTACLE_DETECTION_HPP
