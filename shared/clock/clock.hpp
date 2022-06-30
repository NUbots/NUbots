#ifndef UTILITY_CLOCK_HPP
#define UTILITY_CLOCK_HPP

#include <nuclear>

namespace utility::clock {
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    void update_rtf(const double& rtf);
    extern double custom_rtf;  // Real time factor
}  // namespace utility::clock

#endif  // UTILITY_CLOCK_HPP
