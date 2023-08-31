#ifndef MODULE_STRATEGY_FALLRECOVERY_HPP
#define MODULE_STRATEGY_FALLRECOVERY_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class FallRecovery : public ::extension::behaviour::BehaviourReactor {
    public:
        /// @brief Called by the powerplant to build and setup the FallRecovery reactor.
        explicit FallRecovery(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_FALLRECOVERY_HPP
