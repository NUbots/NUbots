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
#ifndef MODULE_SKILL_KICKWALK_HPP
#define MODULE_SKILL_KICKWALK_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

#include "utility/input/LimbID.hpp"

namespace module::skill {
    using utility::input::LimbID;

    class KickWalk : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Forward velocity during kick (m/s)
            double kick_approach_velocity_x = 0.0;
            /// @brief Lateral velocity during kick (m/s)
            double kick_approach_velocity_y = 0.0;
            /// @brief Time to wait before the next kick (in seconds)
            long new_kick_wait_time = 0.0;
        } cfg;

        /// @brief Time point when the last kick ended. Used to prevent kick spamming.
        NUClear::clock::time_point last_kick_end = NUClear::clock::now();

    public:
        /// @brief Called by the powerplant to build and setup the KickWalk reactor.
        explicit KickWalk(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_KICKWALK_HPP
