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
#ifndef MODULE_STRATEGY_AVOIDROBOT_HPP
#define MODULE_STRATEGY_AVOIDROBOT_HPP

#include <nuclear>
#include <optional>

#include "extension/Behaviour.hpp"

#include "message/planning/WalkPath.hpp"

namespace module::strategy {

    class AvoidRobot : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Allowed threshold between robot to another robot to start avoidance
            double min_distance_threshold = 0.0;

            /// @brief A margin beyond which avoidance can be stopped to prevent oscillation
            double threshold_margin = 0.0;

            /// @brief Speed magnitude used when walking away from nearby robots
            double avoidance_walk_speed = 0.0;

            /// @brief Minimum valid opponent distance used to avoid normalising near-zero vectors
            double min_valid_obstacle_distance = 0.0;

            /// @brief Weight for the lateral (sidestep) component of avoidance
            double lateral_avoidance_weight = 0.0;

            /// @brief Weight for the backward (retreat) component of avoidance
            double retreat_avoidance_weight = 0.0;

            /// @brief Minimum forward x (robot frame) for an obstacle to be considered in-front
            double min_forward_obstacle_x = 0.0;

            /// @brief Distance under which avoidance is forced even if obstacle is not in-front
            double near_field_avoidance_distance = 0.0;


        } cfg;

        /// @brief Whether avoidance mode is currently active
        bool avoid_active = false;

    public:
        /// @brief Called by the powerplant to build and setup the AvoidRobot reactor.
        explicit AvoidRobot(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_AVOIDROBOT_HPP
