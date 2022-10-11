#ifndef MODULE_MOTION_SERVOREQUEST_HPP
#define MODULE_MOTION_SERVOREQUEST_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::motion {

    class ServoRequest : public ::extension::behaviour::BehaviourReactor {
    public:
        /// @brief Called by the powerplant to build and setup the ServoRequest reactor.
        explicit ServoRequest(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::motion

#endif  // MODULE_MOTION_SERVOREQUEST_HPP
