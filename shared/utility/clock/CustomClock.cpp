#include "CustomClock.h"
#include <chrono>

namespace utility {
namespace clock {
    double custom_rtf                                 = 1.0;  // real time factor
    std::chrono::steady_clock::time_point last_update = std::chrono::steady_clock::now();
}  // namespace clock
}  // namespace utility

namespace NUClear {
clock::time_point clock::now() {

    // now is multiplied by the real time factor to sync
    // NUClear with the simulation time
    auto now = std::chrono::steady_clock::now();
    utility::clock::last_update =
        clock::time_point(utility::clock::last_update
                          + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                (now - utility::clock::last_update) * utility::clock::custom_rtf));
    return utility::clock::last_update;
}
}  // namespace NUClear
