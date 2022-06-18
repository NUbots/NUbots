
#include "clock.hpp"

#include <array>
#include <chrono>

namespace utility::clock {

    struct ClockData {
        /// When the clock was last updated under the true time
        NUClear::base_clock::time_point last_update = NUClear::base_clock::now();
        /// Our calculated time when the clock was last updated in simulated time
        NUClear::base_clock::time_point epoch = last_update;
        /// The real time factor of the simulated clock
        double rtf = 1.0;
    };

    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    static std::array<ClockData, 3> data;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    static std::atomic_int active = 0;

    /// Convenience function to cast durations to the base clock duration type so the functions fit on one line
    template <typename T>
    NUClear::base_clock::duration dc(T&& t) {
        return std::chrono::duration_cast<NUClear::base_clock::duration>(t);
    }

    void update_rtf(const double& rtf) {

        auto now = NUClear::base_clock::now();

        // Set up a circular buffer index for the clock data
        int c = active;
        int n = (c + 1) % data.size();

        // Update the next clock element in the list before we set it for reading so the whole struct is complete before
        // we let others read it
        data[n].epoch       = data[c].epoch + dc((now - data[c].last_update) * data[c].rtf);
        data[n].last_update = now;
        data[n].rtf         = rtf;

        // Set active to the clock element we just updated
        active = n;
    }

}  // namespace utility::clock

namespace NUClear {
    clock::time_point clock::now() {

        using namespace utility::clock;  // Using namespace is fine in a function scope

        // Get the current index in a variable in case it changes while we use it
        int c = active;

        // Calculate the time
        return data[c].epoch + dc((NUClear::base_clock::now() - data[c].last_update) * data[c].rtf);
    }

}  // namespace NUClear
