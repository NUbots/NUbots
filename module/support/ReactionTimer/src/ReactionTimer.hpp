#ifndef MODULE_SUPPORT_REACTIONTIMER_HPP
#define MODULE_SUPPORT_REACTIONTIMER_HPP

#include <nuclear>

namespace module::support {

    struct ReactionProfile {
        std::string identifier = "";
        uint64_t total_time    = 0;
        uint64_t count         = 0;
        uint64_t max_time      = std::numeric_limits<uint64_t>::min();
        uint64_t min_time      = std::numeric_limits<uint64_t>::max();
        uint64_t avg_time      = 0;
    };

    class ReactionTimer : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the ReactionTimer reactor.
        explicit ReactionTimer(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Map to store the reaction profiles
        std::map<std::string, ReactionProfile> reaction_profiles;
    };
}  // namespace module::support

#endif  // MODULE_SUPPORT_REACTIONTIMER_HPP
