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

#ifndef MODULE_PLANNING_PLANSMOOTHPATH_HPP
#define MODULE_PLANNING_PLANSMOOTHPATH_HPP

#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::planning {

    class PlanSmoothPath : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Maximum acceleration per update cycle
            double max_acceleration = 0.005;
            /// @brief Maximum translational velocity in x direction
            double max_translational_velocity_x = 0.175;
            /// @brief Maximum translational velocity in y direction
            double max_translational_velocity_y = 0.1;
            /// @brief Maximum angular velocity
            double max_angular_velocity = 0.4;
        } cfg;

        /// @brief Previous walk command for smoothing
        Eigen::Vector3d previous_walk_command = Eigen::Vector3d::Zero();

        /// @brief Last update time for calculating dt
        NUClear::clock::time_point last_update_time = NUClear::clock::now();

        /// @brief Apply acceleration limiting to walk command
        /// @param target_command The desired walk command
        /// @return Smoothed walk command
        Eigen::Vector3d apply_acceleration_limiting(const Eigen::Vector3d& target_command);

        /// @brief Constrain velocity components to maximum limits
        /// @param velocity_command The velocity command to constrain
        /// @return Constrained velocity command
        Eigen::Vector3d constrain_velocity(const Eigen::Vector3d& velocity_command);

    public:
        /// @brief Called by the powerplant to build and setup the PlanSmoothPath reactor.
        explicit PlanSmoothPath(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_PLANSMOOTHPATH_HPP
