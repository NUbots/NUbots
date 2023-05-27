#ifndef MODULE_STRATEGY_KICKTOGOAL_HPP
#define MODULE_STRATEGY_KICKTOGOAL_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class KickToGoal : public ::extension::behaviour::BehaviourReactor {
    public:
        /// @brief Called by the powerplant to build and setup the KickToGoal reactor.
        explicit KickToGoal(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_KICKTOGOAL_HPP
