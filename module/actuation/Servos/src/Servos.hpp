#ifndef MODULE_ACTUATION_K1SERVOS_HPP
#define MODULE_ACTUATION_K1SERVOS_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::actuation {
    class Servos : public ::extension::behaviour::BehaviourReactor {
    public:
        explicit Servos(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::actuation

#endif  // MODULE_ACTUATION_SERVOS_HPP
