#ifndef MODULE_STRATEGY_WALKTOFIELDPOSITION_HPP
#define MODULE_STRATEGY_WALKTOFIELDPOSITION_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class WalkToFieldPosition : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the WalkToFieldPosition reactor.
        explicit WalkToFieldPosition(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_WALKTOBALL_HPP
