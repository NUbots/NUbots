#ifndef MODULE_STRATEGY_FINDOBJECT_HPP
#define MODULE_STRATEGY_FINDOBJECT_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class FindObject : public ::extension::behaviour::BehaviourReactor {

    public:
        /// @brief Called by the powerplant to build and setup the FindObject reactor.
        explicit FindObject(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_FINDOBJECT_HPP
