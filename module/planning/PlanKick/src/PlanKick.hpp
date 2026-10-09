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
#ifndef MODULE_PLANNING_PLANKICK_HPP
#define MODULE_PLANNING_PLANKICK_HPP

#include <Eigen/Core>
#include <extension/Behaviour.hpp>
#include <nuclear>
#include <string>

namespace module::planning {

    class PlanKick : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            double ball_timeout_threshold  = 0.0;
            double ball_distance_threshold = 0.0;
            double ball_angle_threshold    = 0.0;
            double target_angle_threshold  = 0.0;
            /// @brief The offset on goal target, shared with WalkToBall's config
            double goal_target_offset = 0.0;
            /// @brief Kick power used when the target is far away (> 2/3 field length)
            double kick_power_far = 0.0;
            /// @brief Kick power used when the target is a medium distance away (1/3 to 2/3 field length)
            double kick_power_mid = 0.0;
            /// @brief Kick power used when the target is close (< 1/3 field length)
            double kick_power_near = 0.0;
        } cfg;

        /// @brief The position of the goal in field space, the Kick task's target
        Eigen::Vector3d rGFf = Eigen::Vector3d::Zero();

        /// @brief The length of the field, used to band kick power by distance to target
        double field_length = 0.0;

    public:
        /// @brief Called by the powerplant to build and setup the PlanKick reactor.
        explicit PlanKick(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_PLANKICK_HPP
