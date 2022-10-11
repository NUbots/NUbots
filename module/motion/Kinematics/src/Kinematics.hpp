#ifndef MODULE_MOTION_KINEMATICS_HPP
#define MODULE_MOTION_KINEMATICS_HPP

#include <nuclear>

namespace module::motion {

    class Kinematics : public NUClear::Reactor {
    private:
        /// @brief Kinematics model used for calculating inverse kinematics
        message::motion::KinematicsModel kinematicsModel{};

    public:
        /// @brief Called by the powerplant to build and setup the Kinematics reactor.
        explicit Kinematics(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::motion

#endif  // MODULE_MOTION_KINEMATICS_HPP
