#ifndef MODULE_MOTION_KINEMATICS_HPP
#define MODULE_MOTION_KINEMATICS_HPP

#include <extension/Behaviour.hpp>
#include <nuclear>

#include "message/motion/KinematicsModel.hpp"

namespace module::motion {

    class Kinematics : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Kinematics model used for calculating inverse kinematics
        message::motion::KinematicsModel kinematicsModel{};

    public:
        /// @brief Called by the powerplant to build and setup the Kinematics reactor.
        explicit Kinematics(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::motion

#endif  // MODULE_MOTION_KINEMATICS_HPP
