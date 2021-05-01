#ifndef UTILITY_CLOCK_HPP
#define UTILITY_CLOCK_HPP

#include <nuclear>

namespace utility::clock {
    extern double custom_rtf;  // real time factor
    extern std::chrono::steady_clock::time_point last_update;
}  // namespace utility::clock

#endif  // UTILITY_CLOCK_HPP
