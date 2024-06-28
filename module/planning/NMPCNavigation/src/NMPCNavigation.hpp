/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
#ifndef MODULE_PLANNING_NMPCNAVIGATION_HPP
#define MODULE_PLANNING_NMPCNAVIGATION_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nlopt.hpp>
#include <nuclear>
#include <vector>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Robot.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/StandStill.hpp"

#include "utility/math/comparison.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"


namespace module::planning {

    class NMPCNavigation : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Time step for NMPC
            double dt = 0.0;
            /// @brief State cost matrix
            Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
            /// @brief Control cost matrix
            Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
            /// @brief Maximum velocity in x direction
            double max_velocity_x = 0.0;
            /// @brief Maximum velocity in y direction
            double max_velocity_y = 0.0;
            /// @brief Maximum angular velocity
            double max_angular_velocity = 0.0;
            /// @brief Weight for obstacle avoidance
            double obstacle_weight = 0.0;
            /// @brief Robot radius
            double robot_radius = 0.0;
            /// @brief Obstacle radius
            double obstacle_radius = 0.0;
        } cfg;

        static constexpr size_t horizon    = 20;
        static constexpr size_t n_opt_vars = 3 * horizon;


        /**
         * @brief Get the current state of the robot
         * @param sensors The current sensor readings
         * @return Current state vector [x, y, theta]
         */
        Eigen::Vector3d get_current_state(const message::input::Sensors& sensors);

        /**
         * @brief Get the target state from the WalkTo message
         * @param walk_to The WalkTo message
         * @return Target state vector [x, y, theta]
         */
        Eigen::Vector3d get_target_state(const message::planning::WalkTo& walk_to);

        /**
         * @brief Get the obstacles from the Robots message
         * @param robots The Robots message
         * @param sensors The current sensor readings
         * @return Vector of obstacle positions
         */
        std::vector<Eigen::Vector2d> get_obstacles(const std::shared_ptr<const message::localisation::Robots>& robots,
                                                   const message::input::Sensors& sensors);

        /**
         * @brief Solve the NMPC optimization problem
         * @param initial_state The initial state
         * @param target_state The target state
         * @param obstacles The obstacles
         * @return Optimal control actions
         */
        Eigen::VectorXd solve_nmpc(const Eigen::Vector3d& initial_state,
                                   const Eigen::Vector3d& target_state,
                                   const std::vector<Eigen::Vector2d>& obstacles);

        /**
         * @brief Cost function for NMPC optimization
         * @param x The optimization variables
         * @param grad The gradient of the cost function (unused)
         * @param initial_state The initial state
         * @param target_state The target state
         * @param obstacles The obstacles
         * @return The cost
         */
        double cost_function(const Eigen::Matrix<double, n_opt_vars, 1>& x,
                             Eigen::Matrix<double, n_opt_vars, 1>& grad,
                             const Eigen::Vector3d& initial_state,
                             const Eigen::Vector3d& target_state,
                             const std::vector<Eigen::Vector2d>& obstacles);

        /**
         * @brief Emit debug information
         * @param initial_state The initial state
         * @param target_state The target state
         * @param optimal_actions The optimal control actions
         */
        void emit_debug_info(const Eigen::Vector3d& initial_state,
                             const Eigen::Vector3d& target_state,
                             const Eigen::VectorXd& optimal_actions);

    public:
        /// @brief Called by the powerplant to build and setup the NMPCNavigation reactor.
        explicit NMPCNavigation(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_NMPCNAVIGATION_HPP
