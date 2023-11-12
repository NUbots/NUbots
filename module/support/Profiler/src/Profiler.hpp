#ifndef MODULE_SUPPORT_REACTIONTIMER_HPP
#define MODULE_SUPPORT_REACTIONTIMER_HPP

#include <nuclear>

namespace module::support {

    struct ReactionProfile {
        std::string name  = "";
        double total_time = 0;
        uint64_t count    = 0;
        double max_time   = std::numeric_limits<double>::min();
        double min_time   = std::numeric_limits<double>::max();
        double avg_time   = 0;
    };

    class Profiler : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Profiler reactor.
        explicit Profiler(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Map to store the reaction profiles
        std::map<std::string, ReactionProfile> reaction_profiles;
    };
}  // namespace module::support

#endif  // MODULE_SUPPORT_REACTIONTIMER_HPP
