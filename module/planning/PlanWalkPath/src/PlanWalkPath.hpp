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

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::planning {

    class PlanWalkPath : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            double enter_rotate_to_target  = 0.0;
            double exit_rotate_to_target   = 0.0;
            double enter_walk_to_target    = 0.0;
            double exit_walk_to_target     = 0.0;
            double enter_rotate_to_heading = 0.0;
            double exit_rotate_to_heading  = 0.0;
            /// @brief Maximum walk command velocity magnitude for walking to ball
            double max_translational_velocity_magnitude = 0;
            /// @brief Minimum walk command velocity for walking to ball
            double min_translational_velocity_magnitude = 0;
            /// @brief Crude acceleration, the maximum increment/decrease in walk command velocity per update
            double acceleration = 0;
            /// @brief Region around ball to begin decelerating in
            double approach_radius = 0;
            /// @brief Maximum angular velocity command for walking to ball
            double max_angular_velocity = 0;
            /// @brief Minimum angular velocity command for walking to ball
            double min_angular_velocity = 0;
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

        struct State {
            enum Value {
                /// @brief The beginning of the planner, where we determine what state the robot is in
                STOP,
                /// @brief The robot is rotating to face the position it needs to walk to
                /// This is to prevent attempts to strafe and walk backwards to reach the position target
                ROTATE_TO_TARGET,
                /// @brief The robot is walking to the target from a distance
                /// The robot should slow down as it approaches the target
                WALK_TO_TARGET,
                /// @brief The robot is rotating on the spot to face the requested heading
                ROTATE_TO_HEADING,
            } value;
            State(const Value& v) : value(v) {}
            operator int() const {
                return value;
            }
        };

        /// @brief Current magnitude of the translational velocity of the walk command
        double velocity_magnitude = 0;

    public:
        /// @brief Called by the powerplant to build and setup the PlanWalkPath reactor.
        explicit PlanWalkPath(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_PLANWALKPATH_HPP
