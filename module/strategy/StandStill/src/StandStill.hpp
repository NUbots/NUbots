#ifndef MODULE_STRATEGY_STANDSTILL_HPP
#define MODULE_STRATEGY_STANDSTILL_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class StandStill : public ::extension::behaviour::BehaviourReactor {

    public:
        /// @brief Called by the powerplant to build and setup the StandStill reactor.
        explicit StandStill(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_STANDSTILL_HPP
