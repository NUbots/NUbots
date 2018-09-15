/*#ifndef UTILITY_CLOCK_H
#define UTILITY_CLOCK_H

#include <nuclear>

namespace utility {
    namespace clock {
    extern double custom_rtf; // real time factor
    extern std::chrono::steady_clock::time_point lastUpdate;
    }
}

// namespace NUClear {
//     // extern clock::time_point clock::now();
//     clock::time_point clock::now() {

//     // now is multiplied by the real time factor to sync
//     // NUClear with the simulation time
//     auto now = std::chrono::steady_clock::now();
//     return clock::time_point(std::chrono::duration_cast<std::chrono::steady_clock::duration>(now.time_since_epoch() * utility::clock::custom_rtf));
// }
// }  // namespace NUClear

#endif // UTILITY_CLOCK_H*/
