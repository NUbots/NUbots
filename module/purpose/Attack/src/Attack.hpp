#ifndef MODULE_PURPOSE_ATTACK_HPP
#define MODULE_PURPOSE_ATTACK_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    class Attack : public ::extension::behaviour::BehaviourReactor {
    public:
        /// @brief Called by the powerplant to build and setup the Attack reactor.
        explicit Attack(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_ATTACK_HPP
