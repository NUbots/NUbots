#ifndef MODULE_ACTUATION_KINEMATICS_HPP
#define MODULE_ACTUATION_KINEMATICS_HPP

#include <extension/Behaviour.hpp>
#include <nuclear>
#include <tinyrobotics/inversekinematics.hpp>
#include <tinyrobotics/kinematics.hpp>
#include <tinyrobotics/math.hpp>
#include <tinyrobotics/parser.hpp>

#include "message/actuation/KinematicsModel.hpp"

namespace module::actuation {

    using namespace tinyrobotics;

    class Kinematics : public ::extension::behaviour::BehaviourReactor {
    public:
        /// @brief Called by the powerplant to build and setup the Kinematics reactor.
        explicit Kinematics(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Number of actuatable joints in the NUgus robot
        static const int n_joints = 20;

        /// @brief tinyrobotics model of the NUgus robot
        tinyrobotics::Model<double, n_joints> nugus_model_left;
        tinyrobotics::Model<double, n_joints> nugus_model_right;

        /// @brief tinyrobotics inverse kinematics options
        InverseKinematicsOptions<double, n_joints> options;

    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Path to the URDF file
            std::string urdf_path;
        } cfg;
    };

}  // namespace module::actuation

#endif  // MODULE_ACTUATION_KINEMATICS_HPP
