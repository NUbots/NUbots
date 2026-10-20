#ifndef MODULE_ACTUATION_K1SERVOS_HPP
#define MODULE_ACTUATION_K1SERVOS_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::actuation {
    class K1Servos : public ::extension::behaviour::BehaviourReactor {
    public:
        explicit K1Servos(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::actuation

#endif  // MODULE_ACTUATION_K1SERVOS_HPP
