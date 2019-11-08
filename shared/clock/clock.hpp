#ifndef UTILITY_CLOCK_H
#define UTILITY_CLOCK_H

#include <nuclear>

namespace utility {
namespace clock {
    extern double custom_rtf;  // real time factor
    extern std::chrono::steady_clock::time_point last_Update;
}  // namespace clock
}  // namespace utility

#endif  // UTILITY_CLOCK_H
