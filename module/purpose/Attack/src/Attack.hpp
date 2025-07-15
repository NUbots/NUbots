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
#ifndef MODULE_PURPOSE_ATTACK_HPP
#define MODULE_PURPOSE_ATTACK_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    class Attack : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Tracks the time the current team has had the ball (seconds)
        NUClear::clock::duration possession_duration = NUClear::clock::duration::zero();

        /// @brief Last timestamp when we received an attack message (seconds)
        NUClear::clock::time_point last_timestamp;

        /// @brief Whether or not we are in possession
        bool in_possession = false;

        /// @brief Stores configuration values
        struct Config {
            /// @brief When to kick the ball - "Never", "Always", "AttackingThird"
            std::string kick_when = "";  // Default to kicking in the attacking third
            /// @brief The amount of time the robot should think the opponent has had the ball before tackling
            NUClear::clock::duration possession_timeout{};
        } cfg;

        // function to handle the buffer of team possession time
        void confirm_possession(bool in_possession_proposal) {
            // get the time since the last attack message
            auto now = NUClear::clock::now();
            auto time_since_last_attack = now - last_timestamp;
            last_timestamp = now;

            if (in_possession == in_possession_proposal) {
                // If we are in possession and the proposal is also in possession, reset to zero
                possession_duration = NUClear::clock::duration::zero();
                log<DEBUG>("reset");
            }
            else {
                // If we were in possession but the proposal is not, we reset the buffer
                possession_duration += time_since_last_attack;
                log<DEBUG>("adding");
            }

            if (possession_duration > cfg.possession_timeout) {
                // If the possession duration is greater than the timeout, we have swapped possession
                in_possession = in_possession_proposal;
                log<DEBUG>("swapped");
            }
        }

    public:
        /// @brief Called by the powerplant to build and setup the Attack reactor.
        explicit Attack(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_ATTACK_HPP
