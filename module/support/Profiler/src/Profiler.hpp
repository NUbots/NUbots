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
#ifndef MODULE_SUPPORT_PROFILER_HPP
#define MODULE_SUPPORT_PROFILER_HPP

#include <nuclear>

#include "message/support/nuclear/ReactionProfile.hpp"

namespace module::support {

    using message::support::nuclear::ReactionProfile;

    class Profiler : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Profiler reactor.
        explicit Profiler(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Map to store the reaction profiles
        std::map<uint64_t, ReactionProfile> reaction_profiles;

        /// @brief Total time of all reactions
        double total_time_all = 0;

        /// @brief Total count of all reactions
        int total_count = 0;
    };
}  // namespace module::support

#endif  // MODULE_SUPPORT_PROFILER_HPP
