#ifndef UTILITY_CLOCK_HPP
#define UTILITY_CLOCK_HPP

#include <nuclear>

namespace utility::clock {
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    extern double custom_rtf;  // real time factor
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    extern std::chrono::steady_clock::time_point last_update;
}  // namespace utility::clock

#endif  // UTILITY_CLOCK_HPP
