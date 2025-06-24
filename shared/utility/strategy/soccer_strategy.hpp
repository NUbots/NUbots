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
// todo remove after testing
#include <nuclear>

#include "message/localisation/Robot.hpp"
#include "message/strategy/Who.hpp"

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
     *  @param equidistant_threshold maximum distance for two robots to be considered equidistant
     *  @param self_id the ID of the robot that is calling this function
     *  @param ignore_ids a list of robot IDs to ignore when determining the closest robot
     *  @param include_opponents whether to include opponents in the search
     *
     *  @return A vector of pairs, these contain a Possession type and distance to the ball.
     */
    std::pair<unsigned int, double> get_closest_bot(const Eigen::Vector3d& rBWw,
                                                    const Robots& robots,
                                                    const Eigen::Isometry3d& Hfw,
                                                    const Eigen::Isometry3d& Hrw,
                                                    double equidistant_threshold,
                                                    unsigned int self_id,
                                                    std::vector<unsigned int> const& ignore_ids,
                                                    bool include_opponents = true) {
        // Transform ball position to field coordinates
        Eigen::Vector3d rBFf         = Hfw * rBWw;
        Eigen::Vector3d rRFf         = (Hfw * Hrw.inverse()).translation();
        double self_distance_to_ball = (rRFf - rBFf).head<2>().norm();

        // Initialise to self
        std::pair<Who, double> closest = {Who{Who::SELF}, self_distance_to_ball};
        unsigned int lowest_id         = self_id;

        // Loop through each robot
        for (const auto& robot : robots.robots) {
            // Skip if not a teammate and not including opponents
            if (!robot.teammate && !include_opponents) {
                continue;
            }
            // Skip robots that are in the ignore list
            if (std::find(ignore_ids.begin(), ignore_ids.end(), robot.purpose.player_id) != ignore_ids.end()) {
                continue;
            }

            // Calculate distance to the ball
            double distance_to_ball = ((Hfw * robot.rRWw) - rBFf).head<2>().norm();
            // Check if close to the closest robot
            bool equidistant = std::abs(distance_to_ball - closest.second) < equidistant_threshold;

            // Opponents that are equidistant win
            if (equidistant && !robot.teammate) {
                closest   = {Who{Who::OPPONENT}, distance_to_ball};
                lowest_id = 0;
            }
            // Equidistant teammates with a lower ID win
            else if (equidistant && (robot.purpose.player_id < lowest_id)) {
                // If it is equidistant and a teammate, lowest ID wins
                closest   = {Who{Who::TEAMMATE}, distance_to_ball};
                lowest_id = robot.purpose.player_id;
            }
            // Closer than equidistant wins
            else if (!equidistant && (distance_to_ball < closest.second)) {
                closest   = {robot.teammate ? Who{Who::TEAMMATE} : Who{Who::OPPONENT}, distance_to_ball};
                lowest_id = robot.teammate ? robot.purpose.player_id : 0;
            }
        }

        return {lowest_id, closest.second};
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
     * @param equidistant_threshold maximum distance for two robots to be considered equidistant
     * @param self_id the ID of the robot that is calling this function
     * @param ignore_ids a list of robot IDs to ignore when determining possession
     *
     * @return A possession value which determines if any robot (or NONE) has the ball.
     */
    Who ball_possession(const Eigen::Vector3d& rBWw,
                        const Robots& robots,
                        const Eigen::Isometry3d& Hfw,
                        const Eigen::Isometry3d& Hrw,
                        double threshold,
                        double equidistant_threshold,
                        unsigned int self_id,
                        std::vector<unsigned int> const& ignore_ids) {
        // Function determines who has possession based on proximity and a threshold distance.
        auto closest = get_closest_bot(rBWw, robots.robots, Hfw, Hrw, equidistant_threshold, self_id, ignore_ids);

        // 'closest.second' is the distance to the ball
        // If the distance is greater than the threshold, then the robot is too far
        // for possession.
        if (closest.second > threshold) {
            return Who{Who::NONE};
        }
        // If robot is close to ball, robot that is closest will possess the ball.
        return (closest.first == self_id) ? Who::SELF : (closest.first == 0 ? Who::OPPONENT : Who::TEAMMATE);
    }

    /**
     * @brief Determines if we are the closest robot to the ball on our team
     *
     * @param rBWw ball position in world coordinates
     * @param robots localisation of every known robot in the game
     * @param Hfw transformation of world to field coordinates
     * @param Hrw transformation of world to robot coordinates
     * @param equidistant_threshold maximum distance to the ball to be considered equidistant
     * @param self_id the ID of the robot that is calling this function
     * @param ignore_ids a list of robot IDs to ignore when determining the closest robot
     *
     * @return true if we are the closest robot to the ball on our team, false otherwise
     */
    unsigned int closest_to_ball_on_team(const Eigen::Vector3d& rBWw,
                                         const Robots& robots,
                                         const Eigen::Isometry3d& Hfw,
                                         const Eigen::Isometry3d& Hrw,
                                         double equidistant_threshold,
                                         unsigned int self_id,
                                         std::vector<unsigned int> const& ignore_ids) {
        // Function determines who has possession based on proximity and a threshold distance.
        // Exclude opponents from the search
        auto closest =
            get_closest_bot(rBWw, robots.robots, Hfw, Hrw, equidistant_threshold, self_id, ignore_ids, false);

        // Return true if the closest robot is us
        return closest.first;
    }

    /**
     * @brief Determines if we are the furthest back robot on our team
     *
     * @param robots localisation of every known robot in the game
     * @param Hfw transformation of world to field coordinates
     * @param Hrw transformation of world to robot coordinates
     * @param equidistant_threshold maximum distance to the ball to be considered equidistant
     * @param self_id the ID of the robot that is calling this function
     * @param ignore_ids a list of robot IDs to ignore when determining the robot that is furthest back
     *
     * @return true if we are the furthest back robot on our team, false otherwise
     */
    bool furthest_back(const Robots& robots,
                       const Eigen::Isometry3d& Hfw,
                       const Eigen::Isometry3d& Hrw,
                       double equidistant_threshold,
                       unsigned int self_id,
                       std::vector<unsigned int> const& ignore_ids) {
        // Transform our position to field coordinates
        Eigen::Vector3d rRFf = (Hfw * Hrw.inverse()).translation();
        double furthest      = std::abs(rRFf.y());

        // Look for any teammates that are further back than us
        for (const auto& robot : robots.robots) {
            // Only consider non penalised teammates
            bool ignore_id =
                std::find(ignore_ids.begin(), ignore_ids.end(), robot.purpose.player_id) != ignore_ids.end();
            if ((robot.teammate && robot.purpose.active) && !ignore_id) {
                // Transform robot position to field coordinates
                Eigen::Vector3d rRFf = Hfw * robot.rRWw;

                // Check if equidistant to us
                bool equidistant = std::abs(std::abs(rRFf.y()) - furthest) < equidistant_threshold;
                // If the robot is further back than us, or equidistant and has a higher ID, then we are not the
                // furthest back
                if ((!equidistant && (std::abs(rRFf.y()) > furthest))
                    || (equidistant && (robot.purpose.player_id > self_id))) {
                    return false;
                }
            }
        }

        // If we didn't find any teammates that are further back than us, then we are the furthest back
        return true;
    }
}  // namespace utility::strategy

#endif  // UTILITY_STRATEGY_SOCCER_STRATEGY_HPP
