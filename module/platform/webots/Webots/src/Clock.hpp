#ifndef MODULE_PLATFORM_WEBOTS_CLOCK_HPP
#define MODULE_PLATFORM_WEBOTS_CLOCK_HPP

#include "CompileParams.hpp"

#include <ratio>
#include <chrono>

// Satisfies the Clock type trait
// See https://en.cppreference.com/w/cpp/named_req/Clock

namespace module::platform::webots {
class Clock {
public:
    using rep = int;
    using period = std::ratio<1, 1000>; // Should be the same as the webots world
    using duration = std::chrono::duration<rep, period>;
    using time_point = std::chrono::time_point<Clock>;

    static constexpr bool is_steady = false;

    static time_point now();

    static void tick();

private:
    static duration current_tick;
};

// C++20
// At compile time check that this type actually does meet the Clock type trait
//static_assert(std::chrono::is_clock<Clock>::value, "The Webots clock did not satisfy the clock type trait.");
}

#endif //MODULE_PLATFORM_WEBOTS_CLOCK_HPP
