#ifndef MODULE_STRATEGY_STANDSTILL_HPP
#define MODULE_STRATEGY_STANDSTILL_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class StandStill : public ::extension::behaviour::BehaviourReactor {

    private:
        /// @brief Whether we are currently standing
        /// This is used to prevent the robot continually calling the stand script, even when it is already in the
        /// standing position
        bool standing = false;

    public:
        /// @brief Called by the powerplant to build and setup the StandStill reactor.
        explicit StandStill(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_STANDSTILL_HPP
