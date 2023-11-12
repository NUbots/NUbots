#ifndef MODULE_SUPPORT_REACTIONTIMER_HPP
#define MODULE_SUPPORT_REACTIONTIMER_HPP

#include <nuclear>

namespace module::support {

    struct ReactionProfile {
        uint64_t reaction_id = 0;
        double total_time    = 0;
        uint64_t count       = 0;
        double max_time      = std::numeric_limits<double>::min();
        double min_time      = std::numeric_limits<double>::max();
        double avg_time      = 0;
        double percentage    = 0;
    };

    class Profiler : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Profiler reactor.
        explicit Profiler(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Map to store the reaction profiles
        std::map<uint64_t, ReactionProfile> reaction_profiles;

        /// @brief The start time of the profiler
        NUClear::clock::time_point start_time;
    };
}  // namespace module::support

#endif  // MODULE_SUPPORT_REACTIONTIMER_HPP
