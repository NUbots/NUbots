#ifndef MODULE_SUPPORT_REACTIONTIMER_HPP
#define MODULE_SUPPORT_REACTIONTIMER_HPP

#include <nuclear>

#include "message/support/nuclear/ReactionProfile.hpp"

namespace module::support {

    using message::support::nuclear::ReactionProfile;

    class Profiler : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Profiler reactor.
        explicit Profiler(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Map to store the reaction profiles
        std::map<uint64_t, ReactionProfile> reaction_profiles;
    };
}  // namespace module::support

#endif  // MODULE_SUPPORT_REACTIONTIMER_HPP
