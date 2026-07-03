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
#ifndef MODULE_PLANNING_PLANWALKPATH_HPP
#define MODULE_PLANNING_PLANWALKPATH_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

#include "walk_path_control.hpp"

#include "extension/Behaviour.hpp"

#include "message/behaviour/state/Stability.hpp"

namespace module::planning {

    class PlanWalkPath : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief WalkTo controller parameters (velocity limits, gains, alignment regions)
            walk_path::WalkToParams walk_to{};
            /// @brief Minimum effective velocity command per axis, smaller nonzero commands are bumped up to these
            Eigen::Vector3d min_velocity = Eigen::Vector3d::Zero();
            /// @brief Smoothed commands below this per-axis magnitude are snapped to zero
            Eigen::Vector3d zero_tolerance = Eigen::Vector3d::Zero();

            /// @brief Rotate on spot walk command angular velocity
            double rotate_velocity = 0.0;
            /// @brief Rotate on spot walk command forward velocity
            double rotate_velocity_x = 0.0;
            /// @brief Rotate on spot walk command side velocity
            double rotate_velocity_y = 0.0;

            /// @brief Pivot around point angular velocity (rad/s)
            double pivot_angular_velocity = 0.0;
            /// @brief Radius of the orbit around the pivot point (m)
            double pivot_radius = 0.0;
            /// @brief Forward velocity bias while pivoting (m/s)
            double pivot_forward_velocity = 0.0;

            /// @brief Radius to avoid obstacles
            double obstacle_radius = 0.0;

            /// @brief Exponential smoothing time constant for the [x,y,theta]-velocity
            /// @note  Set to [0, 0, 0] to functionally disable smoothing
            Eigen::Vector3d tau = Eigen::Vector3d(0, 0, 0);
            /// @brief Exponential smoothing factor for the velocity command [x, y, theta]
            Eigen::Vector3d alpha = Eigen::Vector3d(1, 1, 1);
            /// @brief Complementary exponential smoothing factor for the velocity command [x, y, theta]
            Eigen::Vector3d one_minus_alpha = Eigen::Vector3d::Ones() - alpha;

        } cfg;

        /// @brief Previous (smoothed, pre-dead-zone) walk command
        Eigen::Vector3d previous_walk_command = Eigen::Vector3d::Zero();

        /// @brief Update frequency of the walk command smoothing
        static constexpr int UPDATE_FREQUENCY = 10;

        /// @brief Current stability of the robot
        message::behaviour::state::Stability stability{};

        /// @brief Gets the closest obstacle in the path to the target, including obstacles close to that obstacle
        /// @param all_obstacles vector of all obstacles in the world
        /// @param rDRr vector from robot to final target
        /// @return vector of closest obstacle in the path to avoid and its neighbours
        const std::vector<Eigen::Vector2d> get_obstacles(const std::vector<Eigen::Vector2d>& all_obstacles,
                                                         const Eigen::Vector2d& rDRr);

    public:
        /// @brief Called by the powerplant to build and setup the PlanWalkPath reactor.
        explicit PlanWalkPath(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_PLANWALKPATH_HPP
