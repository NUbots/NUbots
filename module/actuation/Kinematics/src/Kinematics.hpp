#ifndef MODULE_ACTUATION_KINEMATICS_HPP
#define MODULE_ACTUATION_KINEMATICS_HPP

#include <extension/Behaviour.hpp>
#include <nuclear>

#include "message/actuation/KinematicsModel.hpp"

namespace module::actuation {

    class Kinematics : public ::extension::behaviour::BehaviourReactor {
    public:
        /// @brief Called by the powerplant to build and setup the Kinematics reactor.
        explicit Kinematics(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::actuation

#endif  // MODULE_ACTUATION_KINEMATICS_HPP
