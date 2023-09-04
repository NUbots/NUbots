#ifndef MODULE_ACTUATION_KINEMATICS_HPP
#define MODULE_ACTUATION_KINEMATICS_HPP

#include <extension/Behaviour.hpp>
#include <nuclear>
#include <tinyrobotics/inversekinematics.hpp>
#include <tinyrobotics/kinematics.hpp>
#include <tinyrobotics/math.hpp>
#include <tinyrobotics/parser.hpp>

#include "message/actuation/KinematicsModel.hpp"
#include "message/actuation/Limbs.hpp"
#include "message/actuation/ServoCommand.hpp"

#include "utility/input/ServoID.hpp"

namespace module::actuation {

    using tinyrobotics::InverseKinematicsMethod;

    class Kinematics : public ::extension::behaviour::BehaviourReactor {
    public:
        /// @brief Called by the powerplant to build and setup the Kinematics reactor.
        explicit Kinematics(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Number of actuatable joints in the NUgus robot
        static const int n_joints = 20;

        /// @brief tinyrobotics model of the NUgus robot for left leg IK
        tinyrobotics::Model<double, n_joints> nugus_model_left;

        /// @brief tinyrobotics model of the NUgus robot for right leg IK
        tinyrobotics::Model<double, n_joints> nugus_model_right;

        /// @brief tinyrobotics inverse kinematics options
        tinyrobotics::InverseKinematicsOptions<double, n_joints> options;

        /**
         * @brief Converts a string to an InverseKinematicsMethod
         *
         * @param method_string
         * @return InverseKinematicsMethod
         */
        InverseKinematicsMethod ik_string_to_method(const std::string& method_string);

    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Path to the URDF file
            std::string urdf_path = "";

            /// @brief Name of torso link in tinyrobotics model
            std::string torso_name = "torso";

            /// @brief Name of left foot base link in tinyrobotics model
            std::string left_foot_name = "left_foot_base";

            /// @brief Name of right foot base link in tinyrobotics model
            std::string right_foot_name = "right_foot_base";
        } cfg;
    };

}  // namespace module::actuation

#endif  // MODULE_ACTUATION_KINEMATICS_HPP
