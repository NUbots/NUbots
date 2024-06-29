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

#include "extension/Behaviour.hpp"

namespace module::planning {

    class PlanWalkPath : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Maximum walk command velocity x for walking
            double max_translational_velocity_x = 0.0;
            /// @brief Maximum walk command velocity y for walking
            double max_translational_velocity_y = 0.0;
            /// @brief Maximum angular velocity command for walking
            double max_angular_velocity = 0.0;
            /// @brief Crude acceleration, the maximum increment/decrease in walk command velocity per update
            double acceleration = 0.0;
            // Distance to target point to begin decelerating and aligning with target heading
            double max_align_radius = 0.0;
            // Distance to target point to begin decelerating
            double min_align_radius = 0.0;
            // Maximum error in orientation to target heading for no translational velocity
            double max_angle_error = 0.0;
            // Minimum error in orientation to target heading for maximum translational velocity
            double min_angle_error = 0.0;
            // Proportional gain for strafing to target point
            double strafe_gain = 0.0;

            /// @brief Rotate on spot walk command angular velocity
            double rotate_velocity = 0.0;
            /// @brief Rotate on spot walk command forward velocity
            double rotate_velocity_x = 0.0;
            /// @brief Rotate on spot walk command side velocity
            double rotate_velocity_y = 0.0;

            /// @brief Pivot ball command angular velocity
            double pivot_ball_velocity = 0.0;
            /// @brief Pivot ball forward velocity
            double pivot_ball_velocity_x = 0.0;
            /// @brief Pivot ball side velocity
            double pivot_ball_velocity_y = 0.0;
        } cfg;

        /// @brief Current magnitude of the translational velocity of the walk command
        double velocity_magnitude = 0.0;

        /// @brief Maximum velocity magnitude of the walk command to clamp "acceleration"
        double max_velocity_magnitude = 0.0;

        /// @brief Norm of translational error from robot to target
        double translational_error = 0.0;

        /// @brief Angle between robot and target point
        double angle_to_target = 0.0;

        /// @brief Angle between robot and target heading
        double angle_to_desired_heading = 0.0;

        /**
         * @brief Constrain a velocity vector to ensure it is within the limits
         * @param v velocity vector to constrain
         * @param vx_max maximum translational velocity in the x direction
         * @param vy_max maximum translational velocity in the y direction
         * @param w_max maximum angular velocity
         * @return Constrained velocity vector
         */
        Eigen::Vector3d constrain_velocity(const Eigen::Vector3d& v, double vx_max, double vy_max, double w_max) {
            Eigen::Vector2d translational_velocity = v.head<2>();
            // If either translational component exceeds the limit, scale the vector to fit within the limits
            if (std::abs(v.x()) >= vx_max || std::abs(v.y()) >= vy_max) {
                double sx = v.x() != 0 ? vx_max / std::abs(v.x()) : 0;
                double sy = v.y() != 0 ? vy_max / std::abs(v.y()) : 0;
                // Select the minimum scale factor to ensure neither limit is exceeded but direction is maintained
                double s               = std::min(sx, sy);
                translational_velocity = v.head<2>() * s;
            }
            // Ensure the angular velocity is within the limits
            double angular_velocity = std::clamp(v.z(), -w_max, w_max);
            return Eigen::Vector3d(translational_velocity.x(), translational_velocity.y(), angular_velocity);
        }

    public:
        /// @brief Called by the powerplant to build and setup the PlanWalkPath reactor.
        explicit PlanWalkPath(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_PLANWALKPATH_HPP
