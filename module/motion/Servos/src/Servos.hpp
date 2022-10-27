#ifndef MODULE_MOTION_SERVOS_HPP
#define MODULE_MOTION_SERVOS_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::motion {

    class Servos : public ::extension::behaviour::BehaviourReactor {
    public:
        /// @brief Called by the powerplant to build and setup the Servos reactor.
        explicit Servos(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::motion

#endif  // MODULE_MOTION_SERVOS_HPP
