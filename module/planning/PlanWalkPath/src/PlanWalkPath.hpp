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

    struct ErrorThresholds {
        double enter_pos_threshold;
        double enter_ori_threshold;
        double leave_pos_threshold;
        double leave_ori_threshold;
    };

    class PlanWalkPath : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            double enter_rotate_to_target_pos_threshold = 0;
            double enter_rotate_to_target_ori_threshold = 0;
            double leave_rotate_to_target_pos_threshold = 0;
            double leave_rotate_to_target_ori_threshold = 0;

            double enter_walk_to_target_pos_threshold = 0;
            double enter_walk_to_target_ori_threshold = 0;
            double leave_walk_to_target_pos_threshold = 0;
            double leave_walk_to_target_ori_threshold = 0;

            double enter_strafe_to_target_pos_threshold = 0;
            double enter_strafe_to_target_ori_threshold = 0;
            double leave_strafe_to_target_pos_threshold = 0;
            double leave_strafe_to_target_ori_threshold = 0;

            double enter_align_with_target_pos_threshold = 0;
            double enter_align_with_target_ori_threshold = 0;
            double leave_align_with_target_pos_threshold = 0;
            double leave_align_with_target_ori_threshold = 0;


            // Distance to target point to begin decelerating
            double approach_radius = 0;

            /// @brief Maximum angular velocity command for walking to ball
            double max_angular_velocity = 0;
            /// @brief Minimum angular velocity command for walking to ball
            double min_angular_velocity = 0;
            /// @brief Maximum walk command velocity magnitude for walking to ball
            double max_translational_velocity_magnitude = 0;
            /// @brief Minimum walk command velocity for walking to ball
            double min_translational_velocity_magnitude = 0;
            /// @brief Crude acceleration, the maximum increment/decrease in walk command velocity per update
            double acceleration = 0;
            /// @brief Rotate on spot walk command angular velocity
            double rotate_velocity = 0;
            /// @brief Rotate on spot walk command forward velocity
            double rotate_velocity_x = 0;
            /// @brief Rotate on spot walk command side velocity
            double rotate_velocity_y = 0;
            /// @brief Pivot ball command angular velocity
            double pivot_ball_velocity = 0;
            /// @brief Pivot ball forward velocity
            double pivot_ball_velocity_x = 0;
            /// @brief Pivot ball side velocity
            double pivot_ball_velocity_y = 0;
        } cfg;

        /// @brief Current magnitude of the translational velocity of the walk command
        double velocity_magnitude = 0;

        /// @brief Norm of translational error from robot to target
        double translational_error = 0;

        /// @brief Angle between robot and target point
        double angle_to_target = 0;

        /// @brief Angle between robot and target heading
        double angle_to_desired_heading = 0;

        // Rotate to face target thresholds
        double rotate_to_target_pos_threshold = 0;
        double rotate_to_target_ori_threshold = 0;

        // Walk to target thresholds
        double walk_to_target_pos_threshold = 0;
        double walk_to_target_ori_threshold = 0;

        // Strafe to target thresholds
        double strafe_to_target_pos_threshold = 0;
        double strafe_to_target_ori_threshold = 0;

        // Align to target thresholds
        double align_with_target_pos_threshold = 0;
        double align_with_target_ori_threshold = 0;

        void emit_debug_information(const Eigen::Isometry3d& Hrd, const Eigen::Vector3d& velocity_target);


    public:
        /// @brief Called by the powerplant to build and setup the PlanWalkPath reactor.
        explicit PlanWalkPath(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_PLANWALKPATH_HPP
