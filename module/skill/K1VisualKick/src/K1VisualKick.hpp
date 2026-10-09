/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
#ifndef MODULE_SKILL_K1VISUALKICK_HPP
#define MODULE_SKILL_K1VISUALKICK_HPP

#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

#include "message/booster/BoosterVisualKick.hpp"

namespace module::skill {

    class K1VisualKick : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Which version of Booster's visual kick to use
            message::booster::VisualKickVer version = message::booster::VisualKickVer::V2;
            /// @brief How long to let the kick run before reporting the task as done
            NUClear::clock::duration kick_duration{};
            /// @brief Scales the Kick task's direction magnitude into the SDK's kick power/range field
            double power_scale = 1.0;
            /// @brief The minimum distance the ball moves from initial position to exit VisualKick
            double min_ball_move_distance = 0.1;
            /// @brief Fraction of the way from the robot to the ball to use as the kick reference point (1.0 = ball)
            double ball_reference_fraction = 1.0;
        } cfg;

        /// @brief The time the current visual kick was started
        NUClear::clock::time_point kick_start_time{};

        /// @brief The initial position of the ball
        Eigen::Vector3d initial_ball_position{};

    public:
        /// @brief Called by the powerplant to build and setup the K1VisualKick reactor.
        explicit K1VisualKick(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_K1VISUALKICK_HPP
