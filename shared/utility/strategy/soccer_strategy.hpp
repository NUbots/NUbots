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

#ifndef UTILITY_STRATEGY_SOCCER_STRATEGY_HPP
#define UTILITY_STRATEGY_SOCCER_STRATEGY_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <utility>
#include <vector>

#include "message/strategy/TeamMates.hpp"


namespace utility::strategy {

    using message::strategy::TeamMates;

    /**
     *   @brief Enum to represent the ball possession status.
     */
    struct Possession {
        enum Value { SELF, TEAMMATE, OPPONENT, NONE };
        Value value = Value::SELF;
    };


    /**
     *   @brief Return a list of robots that are sorted by distance to the ball.
     *
     *   @details
     *  This will calculate the distance from the ball to the:
     *      Robot (SELF)
     *      Teammate (TEAMMATE)
     *
     *  The list will be sorted by ascending order of the distance.
     *
     *  @param rBWw ball position in world coordinates.
     *  @param teammates information about teammate positions.
     *  @param Hfw transformation of world to field coordinates.
     *  @param Hrw transformation of world to robot coordinates.
     *
     *  @return A vector of pairs, these contain a Possession type and distance to the ball.
     */
    std::vector<std::pair<Possession, double>> get_sorted_bots(const Eigen::Vector3d& rBWw,
                                                               const TeamMates& teammates,
                                                               const Eigen::Isometry3d& Hfw,
                                                               const Eigen::Isometry3d& Hrw) {
        // Create empty list.
        std::vector<std::pair<Possession, double>> robots{};

        // Transform ball position to field coordinates.
        // 'rBWw' is ball position in world coordinates.
        // 'Hfw' transforms from world to field coordinates.
        // Multiplying these gives 'rBFf', which is the ball's position in field coordinates.
        Eigen::Vector3d rBFf = Hfw * rBWw;

        // Find self distance to ball.
        // 'Hrw' transforms world to robot.
        Eigen::Vector3d rBRr         = Hrw * rBWw;
        double self_distance_to_ball = rBRr.norm();
        robots.push_back({Possession{Possession::SELF}, self_distance_to_ball});

        // Loop through each teammate,
        // subtract ball position (rBFf) from teammates position (rRFf) to get vector between both.
        for (const auto& mate : teammates.teammates) {
            Eigen::Vector3d rRFf    = mate.rRFf;
            double distance_to_ball = (rRFf - rBFf).norm();
            robots.push_back({Possession{Possession::TEAMMATE}, distance_to_ball});
        }

        // Robots are sorted by distance to the ball.
        // Compare distance smallest to largest.
        std::sort(robots.begin(), robots.end(), [](const auto& a, const auto& b) {
            //
            return a.second < b.second;
        });

        // Sorted list of robots is returned.
        return robots;
    }


    /**
     * @brief Establishes which robot is in possession of the ball.
     *
     * @details
     * The robot closest to the ball (either SELF or TEAMMATE) within a given threshold has possession of the ball.
     * If no robot is within the given threshold, NONE is returned.
     *
     * @param rBWw ball position in world coordinates.
     * @param teammates information about teammate positions.
     * @param Hfw transformation of world to field coordinates.
     * @param Hrw transformation of world to robot coordinates.
     * @param threshold maximum distance to the ball to be considered in possession of the ball.
     *
     * @return A possession value which determines if any robot (or NONE) has the ball.
     */
    Possession get_possession(const Eigen::Vector3d& rBWw,
                              const TeamMates& teammates,
                              const Eigen::Isometry3d& Hfw,
                              const Eigen::Isometry3d& Hrw,
                              double threshold) {

        // Function determines who has possession based on proximity and a threshold distance.
        auto sorted_robots = get_sorted_bots(rBWw, teammates, Hfw, Hrw);

        // If no robots are available, no robot will have possession.
        if (sorted_robots.empty()) {
            return Possession{Possession::NONE};
        }

        // Picks the first robot in the sorted list.
        auto& closest_bot = sorted_robots[0];

        // 'closest_bot.second' is the distance to the ball
        // If the distance is greater than the threshold, then the robot is too far
        // for possession.
        if (closest_bot.second > threshold) {
            return Possession{Possession::NONE};
        }
        // If robot is close to ball, robot that is closest will possess the ball.
        return closest_bot.first;
    }
}  // namespace utility::strategy

#endif  // UTILITY_STRATEGY_SOCCER_STRATEGY_HPP
