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
#include "message/strategy/Who.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/euler.hpp"

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
     *  @return The winning robot's ID (0 for an opponent) and its distance to the ball.
     */
    std::pair<unsigned int, double> get_closest_bot(const Eigen::Vector3d& rBWw,
                                                    const Robots& robots,
                                                    const Eigen::Isometry3d& Hfw,
                                                    const Eigen::Isometry3d& Hrw,
                                                    double equidistant_threshold,
                                                    unsigned int self_id,
                                                    std::vector<unsigned int> const& ignore_ids,
                                                    bool include_opponents = true) {
        Eigen::Vector3d rBFf         = Hfw * rBWw;
        Eigen::Vector3d rRFf         = (Hfw * Hrw.inverse()).translation();
        double self_distance_to_ball = (rRFf - rBFf).head<2>().norm();

        struct Candidate {
            // Opponents share id 0 so they always win a tie-break against any (non-zero) teammate id
            unsigned int id;
            double distance_to_ball;
        };

        std::vector<Candidate> candidates{{self_id, self_distance_to_ball}};
        for (const auto& robot : robots.robots) {
            bool ignored =
                std::find(ignore_ids.begin(), ignore_ids.end(), robot.purpose.player_id) != ignore_ids.end();
            if (ignored || (!robot.teammate && !include_opponents)) {
                continue;
            }
            double distance_to_ball = ((Hfw * robot.rRWw) - rBFf).head<2>().norm();
            candidates.push_back({robot.teammate ? robot.purpose.player_id : 0u, distance_to_ball});
        }

        double closest_distance = std::min_element(candidates.begin(),
                                                    candidates.end(),
                                                    [](const Candidate& a, const Candidate& b) {
                                                        return a.distance_to_ball < b.distance_to_ball;
                                                    })
                                      ->distance_to_ball;

        // Equidistance is judged against the true closest distance rather than a running comparison, so it
        // can't chain through a sequence of near-equal robots into a false tie
        Candidate winner = candidates.front();
        for (const auto& candidate : candidates) {
            bool equidistant = std::abs(candidate.distance_to_ball - closest_distance) < equidistant_threshold;
            if (equidistant && candidate.id > winner.id) {
                winner = candidate;
            }
        }

        return {winner.id, winner.distance_to_ball};
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
     * @brief Estimate the point a robot should walk to before it can kick the ball toward the goal: a short
     * distance behind the ball, on the side away from the goal, so the kick has a straight run-up.
     *
     * @param rBFf ball position in field space
     * @param goal_rFf target goal position in field space
     * @param distance_behind_ball how far behind the ball (m) the approach point sits
     *
     * @return the approach point in field space
     */
    Eigen::Vector2d approach_point_behind_ball(const Eigen::Vector2d& rBFf,
                                               const Eigen::Vector2d& goal_rFf,
                                               double distance_behind_ball) {
        return rBFf - (goal_rFf - rBFf).normalized() * distance_behind_ball;
    }

    /**
     * @brief Estimate how long it would take a robot to reach a target point: turn on the spot to face it,
     * then walk straight there at a constant speed.
     *
     * @details Coarse straight-line estimate only - good enough to rank robots, not a real ETA (doesn't
     * model the walk engine's actual accelerate/turn-while-walking behaviour, see PlanWalkPath).
     *
     * @param rTFf target position in field space
     * @param rRFf robot position in field space
     * @param heading robot's current heading (yaw) in field space, radians
     * @param walk_speed assumed walking speed, m/s
     * @param turn_speed assumed turning speed, rad/s
     *
     * @return estimated time to reach the target, in seconds
     */
    double time_to_ball(const Eigen::Vector2d& rTFf,
                        const Eigen::Vector2d& rRFf,
                        double heading,
                        double walk_speed,
                        double turn_speed) {
        Eigen::Vector2d to_target = rTFf - rRFf;
        double distance          = to_target.norm();
        double angle_to_target   = std::atan2(to_target.y(), to_target.x());
        double turn_angle        = std::abs(utility::math::angle::normalise_angle(angle_to_target - heading));
        return turn_angle / turn_speed + distance / walk_speed;
    }

    /**
     * @brief Determines which robot on our team could get in position to kick the ball toward the goal
     * soonest, based on estimated time to the approach point behind the ball rather than raw distance to
     * the ball itself.
     *
     * @details Teammates' heading isn't tracked (see RobotLocalisation), so it's approximated from their
     * velocity direction when moving with purpose, otherwise assumed to already face the target.
     *
     * @param rBWw ball position in world coordinates
     * @param goal_rFf target goal position in field space
     * @param distance_behind_ball how far behind the ball (m) the approach point sits
     * @param robots localisation of every known robot in the game
     * @param Hfw transformation of world to field coordinates
     * @param Hrw transformation of world to robot coordinates
     * @param walk_speed assumed walking speed, m/s
     * @param turn_speed assumed turning speed, rad/s
     * @param equidistant_time_threshold time difference (s) below which two robots are equally fast, with
     * the higher player ID winning the tie (same convention as get_closest_bot)
     * @param self_id the ID of the robot that is calling this function
     * @param ignore_ids a list of robot IDs to ignore when determining who is fastest
     * @param times_out if non-null, filled with the (player ID, estimated time) pair considered for
     * every candidate (self and every non-ignored teammate), for debugging/visualisation
     *
     * @return the ID of the robot judged fastest (opponents are not considered)
     */
    unsigned int fastest_to_ball_on_team(const Eigen::Vector3d& rBWw,
                                         const Eigen::Vector2d& goal_rFf,
                                         double distance_behind_ball,
                                         const Robots& robots,
                                         const Eigen::Isometry3d& Hfw,
                                         const Eigen::Isometry3d& Hrw,
                                         double walk_speed,
                                         double turn_speed,
                                         double equidistant_time_threshold,
                                         unsigned int self_id,
                                         std::vector<unsigned int> const& ignore_ids,
                                         std::vector<std::pair<unsigned int, double>>* times_out = nullptr) {
        Eigen::Vector2d rBFf = (Hfw * rBWw).head<2>();
        Eigen::Vector2d rTFf = approach_point_behind_ball(rBFf, goal_rFf, distance_behind_ball);

        Eigen::Isometry3d Hfr = Hfw * Hrw.inverse();
        Eigen::Vector2d rRFf  = Hfr.translation().head<2>();
        double self_heading   = utility::math::euler::mat_to_rpy_intrinsic(Hfr.rotation()).z();

        struct Candidate {
            unsigned int id;
            double time;
        };

        std::vector<Candidate> candidates{{self_id, time_to_ball(rTFf, rRFf, self_heading, walk_speed, turn_speed)}};

        for (const auto& robot : robots.robots) {
            bool ignored =
                std::find(ignore_ids.begin(), ignore_ids.end(), robot.purpose.player_id) != ignore_ids.end();
            if (!robot.teammate || ignored) {
                continue;
            }

            Eigen::Vector2d teammate_rFf = (Hfw * robot.rRWw).head<2>();

            // Approximate heading from velocity direction when moving with purpose, otherwise assume
            // already facing the target
            Eigen::Vector2d teammate_vFf = (Hfw.linear() * robot.vRw).head<2>();
            Eigen::Vector2d to_target    = rTFf - teammate_rFf;
            double teammate_heading      = teammate_vFf.norm() > 0.2
                                              ? std::atan2(teammate_vFf.y(), teammate_vFf.x())
                                              : std::atan2(to_target.y(), to_target.x());

            double time = time_to_ball(rTFf, teammate_rFf, teammate_heading, walk_speed, turn_speed);
            candidates.push_back({robot.purpose.player_id, time});
        }

        double fastest_time = std::min_element(candidates.begin(),
                                               candidates.end(),
                                               [](const Candidate& a, const Candidate& b) {
                                                   return a.time < b.time;
                                               })
                                  ->time;

        // Judge equidistance against the true fastest time, not a running comparison, so it can't chain
        // through near-equal robots into a false tie (see get_closest_bot)
        Candidate winner = candidates.front();
        for (const auto& candidate : candidates) {
            bool equidistant = std::abs(candidate.time - fastest_time) < equidistant_time_threshold;
            if (equidistant && candidate.id > winner.id) {
                winner = candidate;
            }
        }

        if (times_out != nullptr) {
            for (const auto& candidate : candidates) {
                times_out->emplace_back(candidate.id, candidate.time);
            }
        }

        return winner.id;
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
        double furthest      = rRFf.x();

        // Look for any teammates that are further back than us
        for (const auto& robot : robots.robots) {
            // Only consider non penalised teammates
            bool ignore_id =
                std::find(ignore_ids.begin(), ignore_ids.end(), robot.purpose.player_id) != ignore_ids.end();
            if ((robot.teammate && robot.purpose.active) && !ignore_id) {
                // Transform robot position to field coordinates
                Eigen::Vector3d rRFf_robot = Hfw * robot.rRWw;
                double robot_distance      = rRFf_robot.x();

                // Check if equidistant to us
                bool equidistant = std::abs(robot_distance - furthest) < equidistant_threshold;
                // If the robot is further back than us, we are not the furthest back
                if (!equidistant && (robot_distance > furthest)) {
                    return false;
                }

                // If equidistant, higher ID wins
                if (equidistant && robot.purpose.player_id > self_id) {
                    return false;
                }
            }
        }

        // If we didn't find any teammates that are further back than us, then we are the furthest back
        return true;
    }
}  // namespace utility::strategy

#endif  // UTILITY_STRATEGY_SOCCER_STRATEGY_HPP
