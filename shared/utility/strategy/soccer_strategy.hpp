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

#include "message/localisation/Robot.hpp"
#include "message/strategy/Possession.hpp"

namespace utility::strategy {

    using message::localisation::Robots;
    using message::strategy::Who;

    /**
     *   @brief Return a list of robots that are sorted by distance to the ball.
     *
     *   @details
     *  This will calculate the distance from the ball to the:
     *      Robot (SELF)
     *      Teammate (TEAMMATE)
     *      Opponent (OPPONENT)
     *
     *  The list will be sorted by ascending order of the distance.
     *
     *  @param rBWw ball position in world coordinates.
     *  @param robots information about robot positions.
     *  @param Hfw transformation of world to field coordinates.
     *  @param Hrw transformation of world to robot coordinates.
     *
     *  @return A vector of pairs, these contain a Possession type and distance to the ball.
     */
    std::vector<std::pair<Who, double>> get_closest_bot(const Eigen::Vector3d& rBWw,
                                                        const Robots& robots,
                                                        const Eigen::Isometry3d& Hfw,
                                                        const Eigen::Isometry3d& Hrw,
                                                        bool include_opponents = true) {
        // The players in the game represented by team and distance to ball
        std::vector<std::pair<Who, double>> players{};

        // Transform ball position to field coordinates.
        // 'rBWw' is ball position in world coordinates.
        // 'Hfw' transforms from world to field coordinates.
        // Multiplying these gives 'rBFf', which is the ball's position in field coordinates.
        Eigen::Vector3d rBFf = Hfw * rBWw;

        // Find self distance to ball.
        // 'Hrw' transforms world to robot.
        Eigen::Vector3d rBRr         = Hrw * rBWw;
        double self_distance_to_ball = rBRr.norm();
        players.push_back({Who{Who::SELF}, self_distance_to_ball});

        // Loop through each robot,
        // Subtract ball position (rBFf) from robots position (rRFf) to get vector between both.
        for (const auto& robot : robots.robots) {
            Eigen::Vector3d rRFf    = robot.rRFf;
            double distance_to_ball = (rRFf - rBFf).norm();

            // Skip if not a teammate and not including opponents
            if (robot.teammate_id == 0 && !include_opponents) {
                continue;
            }
            // Add the robot to the list of players
            players.push_back(robot.teammate_id == 0 ? Who{Who::OPPONENT} : Who{Who::TEAMMATE}, distance_to_ball);
        }

        // The robot closest to the ball is returned
        return std::max_element(players.begin(), players.end(), [](const auto& a, const auto& b) {
            return a.second < b.second;
        });
        ;
    }


    /**
     * @brief Establishes which robot is in possession of the ball.
     *
     * @details
     * The robot closest to the ball (either SELF or TEAMMATE) within a given threshold has possession of the ball.
     * If no robot is within the given threshold, NONE is returned.
     *
     * @param rBWw ball position in world coordinates.
     * @param robots localisation of every known robot in the game.
     * @param Hfw transformation of world to field coordinates.
     * @param Hrw transformation of world to robot coordinates.
     * @param threshold maximum distance to the ball to be considered in possession of the ball.
     *
     * @return A possession value which determines if any robot (or NONE) has the ball.
     */
    Who ball_possession(const Eigen::Vector3d& rBWw,
                        const Robots& robots,
                        const Eigen::Isometry3d& Hfw,
                        const Eigen::Isometry3d& Hrw,
                        double threshold) {
        // Function determines who has possession based on proximity and a threshold distance.
        auto closest_robot = get_closest_bot(rBWw, robots.robots, Hfw, Hrw);

        // 'closest_bot.second' is the distance to the ball
        // If the distance is greater than the threshold, then the robot is too far
        // for possession.
        if (closest_bot.second > threshold) {
            return Who{Who::NONE};
        }
        // If robot is close to ball, robot that is closest will possess the ball.
        return closest_bot.first;
    }

    /**
     * @brief Determines if we are the closest robot to the ball on our team
     *
     * @param rBWw ball position in world coordinates
     * @param robots localisation of every known robot in the game
     * @param Hfw transformation of world to field coordinates
     * @param Hrw transformation of world to robot coordinates
     *
     * @return true if we are the closest robot to the ball on our team, false otherwise
     */
    bool closest_to_ball_on_team(const Eigen::Vector3d& rBWw,
                                 const Robots& robots,
                                 const Eigen::Isometry3d& Hfw,
                                 const Eigen::Isometry3d& Hrw) {
        // Function determines who has possession based on proximity and a threshold distance.
        // Exclude opponents from the search
        auto closest_robot = get_closest_bot(rBWw, robots.robots, Hfw, Hrw, false);

        // Return true if the closest robot is us
        return closest_robot.first == Who{Who::SELF};
    }
}  // namespace utility::strategy

#endif  // UTILITY_STRATEGY_SOCCER_STRATEGY_HPP
