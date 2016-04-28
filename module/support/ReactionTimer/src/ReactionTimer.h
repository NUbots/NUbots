#ifndef MODULE_SUPPORT_REACTIONTIMER_H
#define MODULE_SUPPORT_REACTIONTIMER_H

#include <nuclear>

namespace module {
namespace support {

    class ReactionTimer : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the ReactionTimer reactor.
        explicit ReactionTimer(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_SUPPORT_REACTIONTIMER_H
