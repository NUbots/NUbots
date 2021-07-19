#ifndef MODULE_SUPPORT_REACTIONSTATS_HPP
#define MODULE_SUPPORT_REACTIONSTATS_HPP

#include <nuclear>

namespace module::support {

    class ReactionStats : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the ReactionTimer reactor.
        explicit ReactionStats(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::support

#endif  // MODULE_SUPPORT_REACTIONSTATS_HPP
