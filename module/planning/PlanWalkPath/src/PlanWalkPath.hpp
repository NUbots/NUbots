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
            /// @brief Maximum velocity magnitude of the walk command to clamp "acceleration"
            double max_velocity_magnitude = 0.0;
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

            /// @brief Maximum angle to target for strafing backwards
            double max_strafe_angle = 0.0;
            /// @brief Buffer to prevent oscillation between forwards and backwards movement
            double backward_buffer = 0.0;
            /// @brief Maximum angle that we are considered to be aligned with the final heading
            double max_aligned_angle = 0.0;
            /// @brief Starting velocity when walking backwards
            double starting_velocity = 0.0;
            /// @brief Multiplier for acceleration/deceleration when walking backwards
            double acceleration_multiplier = 0.0;
            /// @brief Backwards walk vector
            Eigen::Vector2d backwards_vector = Eigen::Vector2d::Zero();

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

            /// @brief Radius to avoid obstacles
            double obstacle_radius = 0.0;
        } cfg;

        /// @brief Current magnitude of the translational velocity of the walk command
        double velocity_magnitude = 0.0;

        /// @brief Boolean value to determine if the robot is walking backwards
        bool is_walking_backwards = false;

        /// @brief Just testing
        Eigen::Vector2d walk_around_obstacle(std::vector<Eigen::Vector2d> obstacles, Eigen::Vector2d rDRr);

        /// @brief Constrain a velocity vector to ensure it is within the limits
        /// @param v velocity vector to constrain
        /// @return Constrained velocity vector
        Eigen::Vector3d constrain_velocity(const Eigen::Vector3d& v);

        /// @brief Head towards the target point without rotation
        /// @return desired velocity magnitude
        double strafe_to_target(const double error);

        /// @brief Walk straight backwards when the robot is next to the target
        /// @param desired_direction desired direction to walk- true for backwards, false for forwards
        /// @return adjusted target direction
        Eigen::Vector2d walk_backwards(bool desired_direction);

        /// @brief Accelerate and rotate the robot towards the target point
        /// @return desired velocity magnitude
        double accelerate_to_target(double desired_heading);

        /// @brief Gets the closest obstacle in the path to the target, including obstacles close to that obstacle
        /// @param all_obstacles vector of all obstacles in the world
        /// @param rDRr vector from robot to final target
        /// @return vector of closest obstacle in the path to avoid and its neighbours
        const std::vector<Eigen::Vector2d> get_obstacles(const std::vector<Eigen::Vector2d>& all_obstacles,
                                                         const Eigen::Vector2d& rDRr);

        /// @brief Adjust the target direction to avoid obstacles
        /// @param robots vector of all robots in the world
        /// @param rDRr vector from robot to final target
        /// @param obstacles vector of obstacles in the path to avoid
        /// @return adjusted target direction
        Eigen::Vector2d adjust_target_direction_for_obstacles(Eigen::Vector2d rDRr,
                                                              const std::vector<Eigen::Vector2d>& obstacles);

    public:
        /// @brief Called by the powerplant to build and setup the PlanWalkPath reactor.
        explicit PlanWalkPath(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_PLANWALKPATH_HPP
