/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2021 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_PLATFORM_WEBOTS_CLOCK_HPP
#define MODULE_PLATFORM_WEBOTS_CLOCK_HPP

#include <chrono>
#include <ratio>


// Satisfies the Clock type trait
// See https://en.cppreference.com/w/cpp/named_req/Clock


namespace module::platform::webots {
    class Clock {
    public:
        /// @brief An arithmetic type, which represents the duration
        using rep = uint64_t;
        /// @brief Tick period of the clock in seconds. Should be the same as the webots world
        using period = std::ratio<1, 1000>;
        /// @brief The duration type of the clock
        using duration = std::chrono::duration<rep, period>;
        /// @brief Specifies the type of time point, which uses this clock
        using time_point = std::chrono::time_point<Clock>;
        /// @brief TODO(KipHamiltons): Why is this false?
        static constexpr bool is_steady = false;

        /// @brief Gets current time
        /// @return A time_point object representing the current point in time
        static time_point now();
        /// @brief TODO(CMurtagh)
        static void tick();
        /// @brief The discrete time point which represents the current time
        static duration current_tick;
    };

    // C++20
    // At compile time check that this type actually does meet the Clock type trait
    // static_assert(std::chrono::is_clock<Clock>::value, "The Webots clock did not satisfy the clock type trait.");
}  // namespace module::platform::webots

#endif  // MODULE_PLATFORM_WEBOTS_CLOCK_HPP
