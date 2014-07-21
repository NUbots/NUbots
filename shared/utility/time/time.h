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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_TIME_TIME_H
#define UTILITY_TIME_TIME_H

#include <armadillo>
#include <chrono>
#include <nuclear>

namespace utility {
namespace time {

    inline double TimeDifferenceSeconds(
        std::chrono::system_clock::time_point end_time,
        std::chrono::system_clock::time_point start_time) {
        auto time_diff = end_time - start_time;
        double nano = std::chrono::duration_cast<std::chrono::nanoseconds>(time_diff).count();
        return nano * 1.0e-9;
    }

    inline uint64_t getUtcTimestamp() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(NUClear::clock::now().time_since_epoch()).count();
    }

    template <class T = std::chrono::milliseconds, typename TClock>
    inline uint64_t getUtcTimestamp(const TClock& timePoint) {
        return std::chrono::duration_cast<T>(timePoint.time_since_epoch()).count();
    }

    inline NUClear::clock::duration durationFromSeconds(double seconds) {
        return NUClear::clock::duration(uint64_t(NUClear::clock::period::den * seconds));
    }

}
}
#endif
