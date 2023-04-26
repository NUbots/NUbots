#ifndef MODULE_ACTUATION_KINEMATICS_HPP
#define MODULE_ACTUATION_KINEMATICS_HPP

#include <extension/Behaviour.hpp>
#include <map>
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

    using namespace tinyrobotics;

    using message::actuation::Body;
    using message::actuation::ServoCommand;
    using message::actuation::ServoState;
    using utility::input::ServoID;

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

        /// @brief Checks if a servo exists in the servo map
        bool servo_exists(const std::map<u_int32_t, ServoCommand>& servos_map, ServoID servo_id) {
            return servos_map.find(servo_id) != servos_map.end();
        }

        /// @brief Maps the servo ID to the index in the tinyrobotics model joint vector
        std::vector<std::pair<int, ServoID>> indices = {{11, ServoID::R_HIP_YAW},
                                                        {10, ServoID::R_HIP_ROLL},
                                                        {9, ServoID::R_HIP_PITCH},
                                                        {8, ServoID::R_KNEE},
                                                        {7, ServoID::R_ANKLE_PITCH},
                                                        {6, ServoID::R_ANKLE_ROLL},
                                                        {5, ServoID::L_HIP_YAW},
                                                        {4, ServoID::L_HIP_ROLL},
                                                        {3, ServoID::L_HIP_PITCH},
                                                        {2, ServoID::L_KNEE},
                                                        {1, ServoID::L_ANKLE_PITCH},
                                                        {0, ServoID::L_ANKLE_ROLL}};

        /// @brief Converts a NUgus ServoCommand message to a tinyrobotics joint vector
        template <typename LimbServos>
        Eigen::Matrix<double, 20, 1> servos_to_configuration(const LimbServos* servos) {
            Eigen::Matrix<double, 20, 1> q = Eigen::Matrix<double, 20, 1>::Zero();
            for (const auto& [index, servo_id] : indices) {
                if (servo_exists(servos->servos, servo_id)) {
                    q(index, 0) = servos->servos.at(servo_id).position;
                }
            }
            return q;
        }

        /// @brief Converts a tinyrobotics joint vector to a NUgus ServoCommand message
        template <typename LimbServos>
        void configuration_to_servos(LimbServos* servos, const Eigen::Matrix<double, 20, 1>& q) {
            for (const auto& [index, servo_id] : indices) {
                if (index < q.size() && servo_exists(servos->servos, servo_id)) {
                    servos->servos[servo_id].position = q(index, 0);
                }
            }
        }

    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Path to the URDF file
            std::string urdf_path;
        } cfg;
    };

}  // namespace module::actuation

#endif  // MODULE_ACTUATION_KINEMATICS_HPP
