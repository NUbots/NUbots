
#include "clock.hpp"

#include <chrono>

namespace utility::clock {

    struct clockStruct {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
        NUClear::base_clock::time_point last_update = NUClear::base_clock::now();
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
        NUClear::base_clock::time_point epoch = last_update;
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
        double rtf = 1.0;  // real time factor
    };

    static clockStruct clockArray[2];
    //static clockStruct clockArrayRead;

    static std::atomic_int clockArrayIndex  = 0;
    static std::atomic_int updateClockIndex = 0;

    void update_rtf(const double& new_rtf) {
        auto now = NUClear::clock::now();  // Changed from base_clock

        updateClockIndex = clockArrayIndex ^ 1;  // Switch to other clock in array

        // Since we are updating our rtf we need to advance our state so the new deltas will be calculated properly
        // We set the variables in this specific order to mimise the error that will occur if the threading reads as we
        // are writing By reducing the delta between state and now any changes in rtf will have a minimal change on
        // delta. The old rtf is used since it is assume that since the last update the old rtf was in effect and
        // only now with future calculations the new rtf will be used

        clockArray[updateClockIndex].epoch =
            clockArray[updateClockIndex].epoch
            + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                (now - clockArray[updateClockIndex].last_update)
                * clockArray[updateClockIndex].rtf);  // set before we update the variables
        clockArray[updateClockIndex].last_update = now;
        clockArray[updateClockIndex].rtf         = new_rtf;
        // clockArrayRead                          = clockArray[clockArrayIndex];

        clockArrayIndex ^= 1;
    }

}  // namespace utility::clock

namespace NUClear {
    clock::time_point clock::now() {
        // Move along the time
        return utility::clock::clockArray[utility::clock::clockArrayIndex].epoch
               + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                   (NUClear::base_clock::now()
                    - utility::clock::clockArray[utility::clock::clockArrayIndex].last_update)
                   * utility::clock::clockArray[utility::clock::clockArrayIndex].rtf);
    }

}  // namespace NUClear
