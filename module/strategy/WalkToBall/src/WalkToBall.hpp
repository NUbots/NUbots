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
#ifndef MODULE_STRATEGY_WALKTOBALL_HPP
#define MODULE_STRATEGY_WALKTOBALL_HPP

#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class WalkToBall : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {

            /// @brief Offset to align the ball with the robot's foot
            double ball_y_offset = 0.0;

            /// @brief The offset on goal target
            double goal_target_offset = 0.0;

            /// @brief The distance from ball to walk to if the ball is in a good position
            double ball_kick_distance = 0.0;

            /// @brief The distance from ball to walk to if the ball is in a tricky position
            double ball_approach_distance = 0.0;

            /// @brief Maximum angle error to the ball before we need to walk to approach distance point
            double max_angle_error = 0.0;

            /// @brief Offset from ball in field space to walk to when approaching the ball from in "front"
            Eigen::Vector3d avoid_ball_offset = Eigen::Vector3d::Zero();

            /// @brief Offset during tackling to avoid the opponent
            double avoid_opponent_offset = 0.0;
            /// @brief Offset during tackling to ensure walking to the side of the ball
            double approach_offset = 0.0;
            /// @brief When the robot's angle error is above this, it will walk to an offset position next to the ball
            double tackle_angle_offset = 0.0;

            /// @brief Distance behind the ball to walk to when positioning
            double distance_behind_ball = 0.0;
            /// @brief Radius of circle around ball where an opponent robot is considered to be in front of the ball
            double infront_of_ball_radius = 0.0;
            /// @brief Distance in front of the ball to check for obstacles
            double infront_check_distance = 0.0;

        } cfg;

        std::optional<Eigen::Vector2d> robot_infront_of_path(const std::vector<Eigen::Vector2d>& all_obstacles,
                                                             const Eigen::Vector2d& rBFf);


        /// @brief The position of the goal {g} in field {f} space
        Eigen::Vector3d rGFf = Eigen::Vector3d::Zero();

    public:
        /// @brief Called by the powerplant to build and setup the WalkToBall reactor.
        explicit WalkToBall(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_WALKTOBALL_HPP
