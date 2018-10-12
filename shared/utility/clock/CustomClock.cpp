#include "CustomClock.hpp"
#include <chrono>

namespace utility {
namespace clock {
    double custom_rtf;  // real time factor
    std::chrono::steady_clock::time_point lastUpdate;
}  // namespace clock
}  // namespace utility

namespace NUClear {
clock::time_point clock::now() {

    // now is multiplied by the real time factor to sync
    // NUClear with the simulation time
    auto now = std::chrono::steady_clock::now();
    utility::clock::lastUpdate =
        clock::time_point(utility::clock::lastUpdate
                          + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                (now - utility::clock::lastUpdate) * utility::clock::custom_rtf));
    return utility::clock::lastUpdate;
}
}  // namespace NUClear
