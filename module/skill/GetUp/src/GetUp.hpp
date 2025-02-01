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
#ifndef MODULE_SKILL_GETUP_HPP
#define MODULE_SKILL_GETUP_HPP

#include <nuclear>
#include <string>
#include <vector>

#include "extension/Behaviour.hpp"

namespace module::skill {

    class GetUp : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            int delay_time; // milliseconds
            /// @brief Script sequence to run when getting from lying on the front to standing
            std::vector<std::string> getup_front;
            /// @brief Script sequence to run when getting from lying on the back to standing
            std::vector<std::string> getup_back;
            /// @brief Script sequence to run when getting from lying on the right to standing
            std::vector<std::string> getup_right;
            /// @brief Script sequence to run when getting from lying on the left to standing
            std::vector<std::string> getup_left;
            /// @brief Script sequence to run when already upright
            std::vector<std::string> getup_upright;
            /// @brief Script sequence to run when told to get up while upside down
            std::vector<std::string> getup_upside_down;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the GetUp reactor.
        explicit GetUp(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_GETUP_HPP
