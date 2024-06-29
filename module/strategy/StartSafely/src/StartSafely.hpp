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
#ifndef MODULE_STRATEGY_STARTSAFELY_HPP
#define MODULE_STRATEGY_STARTSAFELY_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

#include "message/actuation/Limbs.hpp"

namespace module::strategy {

    class StartSafely : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief How long to tell the servo to move for in seconds
            int move_time = 0;
            /// @brief Gain for the servos
            double servo_gain = 0.0;
            /// @brief Allowable error for the servos
            double servo_error = 0.0;
            /// @brief The name of the script to load
            std::string script_name = "";
            /// @brief The maximum time to wait for start safely to complete
            double max_timeout = 0.0;
        } cfg{};

        /// @brief The time on startup
        NUClear::clock::time_point startup_time = NUClear::clock::now();

        /// @brief Stores the servo targets
        std::map<int, double> servo_targets = {};

        /// @brief The script to load
        std::shared_ptr<message::actuation::BodySequence> script = nullptr;

        /// @brief The time to tell the servos to move to
        NUClear::clock::time_point destination_time = NUClear::clock::now();

    public:
        /// @brief Called by the powerplant to build and setup the StartSafely reactor.
        explicit StartSafely(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_STARTSAFELY_HPP
