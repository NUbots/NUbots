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
#ifndef MODULE_SOCCERNEW_HPP
#define MODULE_SOCCERNEW_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    struct EnableIdle {};
    struct DisableIdle {};

    class SoccerNew : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Whether or not to force the robot to ignore GameController and play
            bool force_playing = false;
            /// @brief Delay in seconds before the robot starts playing after button press
            int disable_idle_delay = 0;
            /// @brief Whether or not the player is a goalie
            bool is_goalie = false;
            /// @brief Delay in seconds before the robot can start playing after startup
            int startup_delay = 0;
        } cfg;

        /// @brief Idle state of the robot
        bool idle = false;

    public:
        /// @brief Called by the powerplant to build and setup the SoccerNew reactor.
        explicit SoccerNew(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_SOCCERNEW_HPP
