/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_REFLEX_HEADCONTROLLER_HPP
#define MODULES_BEHAVIOUR_REFLEX_HEADCONTROLLER_HPP

#include <Eigen/Core>
#include <nuclear>

namespace module::motion {

    /**
     * Executes a HeadController action.
     *
     * @author Jade Fountain
     */
    class HeadController : public NUClear::Reactor {
    private:
        /// @brief The id registered in the subsumption system for this module
        const size_t subsumption_id;

        /// @brief Stores configuration values
        struct Config {
            Config() = default;
            /// @brief Head controller priority in the subsumption system
            double head_controller_priority = 0;
            /// @brief Motor gain for head commands
            double head_motor_gain = 0;
            /// @brief Motor torque for head commands
            double head_motor_torque = 0;
            /// @brief Smoothing factor for smoothing goal_angles with exponential filter
            double smoothing_factor = 0;
        } cfg;

        /// @brief Updates the priority of the module in the subsumption system
        void update_priority(const double& priority);

    public:
        explicit HeadController(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Stores the latest goal angle from HeadCommand message
        Eigen::Vector2d goal_angles = Eigen::Vector2d::Zero();

        /// @brief Stores angles to be sent to head servos
        Eigen::Vector2d current_angles = Eigen::Vector2d::Zero();

        /// @brief Bool to inidicate if the goal_angles are in robot space or world space (true if world space)
        bool goal_robot_space = true;

        /// @brief Bool to indicate if goal_angles should be smoothed with exponential filter
        bool smooth = true;
    };
}  // namespace module::motion

#endif  // MODULES_BEHAVIOURS_REFLEX_HEADCONTROLLER_HPP
