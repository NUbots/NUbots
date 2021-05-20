
#include "clock.hpp"

#include <chrono>

namespace utility::clock {

    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    double custom_rtf = 1.0;  // real time factor
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    NUClear::base_clock::time_point last_update = NUClear::base_clock::now();

}  // namespace utility::clock

namespace NUClear {
    clock::time_point clock::now() {

        // now is multiplied by the real time factor to sync
        // NUClear with the simulation time
        auto now = NUClear::base_clock::now();
        utility::clock::last_update =
            clock::time_point(utility::clock::last_update
                              + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  (now - utility::clock::last_update) * utility::clock::custom_rtf));
        return utility::clock::last_update;
    }

}  // namespace NUClear
