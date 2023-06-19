/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_ACTUATION_TINYROBOTICS_HPP
#define UTILITY_ACTUATION_TINYROBOTICS_HPP

#include <Eigen/Geometry>
#include <cmath>
#include <nuclear>
#include <vector>

#include "message/actuation/BodySide.hpp"
#include "message/actuation/KinematicsModel.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/input/Sensors.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/angle.hpp"

namespace utility::actuation::tinyrobotics {

    using message::actuation::BodySide;
    using message::actuation::ServoCommand;
    using utility::input::LimbID;
    using utility::input::ServoID;

    using message::input::Sensors;

    /**
     * @brief Checks if a servo exists in a map of servos
     * @param servos_map map of servos
     * @param servo_id ID of the servo to check
     * @return true if the servo exists in the map
     */
    inline bool servo_exists(const std::map<u_int32_t, ServoCommand>& servos_map, ServoID servo_id) {
        return servos_map.find(servo_id) != servos_map.end();
    }

    // clang-format off
    /// @brief Maps the ServoID to the joint index in the tinyrobotics model joint configuration vector
    inline std::vector<std::pair<int, ServoID>> joint_map = {{0, ServoID::L_ANKLE_ROLL},
                                                      {1, ServoID::L_ANKLE_PITCH},
                                                      {2, ServoID::L_KNEE},
                                                      {3, ServoID::L_HIP_PITCH},
                                                      {4, ServoID::L_HIP_ROLL},
                                                      {5, ServoID::L_HIP_YAW},
                                                      {6, ServoID::R_ANKLE_ROLL},
                                                      {7, ServoID::R_ANKLE_PITCH},
                                                      {8, ServoID::R_KNEE},
                                                      {9, ServoID::R_HIP_PITCH},
                                                      {10, ServoID::R_HIP_ROLL},
                                                      {11, ServoID::R_HIP_YAW},
                                                      {12, ServoID::HEAD_PITCH},
                                                      {13, ServoID::HEAD_YAW},
                                                      {14, ServoID::L_ELBOW},
                                                      {15, ServoID::L_SHOULDER_ROLL},
                                                      {16, ServoID::L_SHOULDER_PITCH},
                                                      {17, ServoID::R_ELBOW},
                                                      {18, ServoID::R_SHOULDER_ROLL},
                                                      {19, ServoID::R_SHOULDER_PITCH}};

    /// @brief Maps the ServoID to the associated link index in the tinyrobotics model
    inline std::vector<std::pair<int, ServoID>> link_map = {
                                                      {1, ServoID::L_HIP_YAW},
                                                      {2, ServoID::L_HIP_ROLL},
                                                      {3, ServoID::L_HIP_PITCH},
                                                      {4, ServoID::L_KNEE},
                                                      {5, ServoID::L_ANKLE_PITCH},
                                                      {7, ServoID::L_ANKLE_ROLL},
                                                      {8, ServoID::R_HIP_YAW},
                                                      {9, ServoID::R_HIP_ROLL},
                                                      {10, ServoID::R_HIP_PITCH},
                                                      {11, ServoID::R_KNEE},
                                                      {12, ServoID::R_ANKLE_PITCH},
                                                      {14, ServoID::R_ANKLE_ROLL},
                                                      {15, ServoID::HEAD_YAW},
                                                      {16, ServoID::HEAD_PITCH},
                                                      {17, ServoID::L_SHOULDER_PITCH},
                                                      {18, ServoID::L_SHOULDER_ROLL},
                                                      {19, ServoID::L_ELBOW},
                                                      {20, ServoID::R_SHOULDER_PITCH},
                                                      {21, ServoID::R_SHOULDER_ROLL},
                                                      {22, ServoID::R_ELBOW}};

    // clang-format on

    /**
     * @brief Converts a Servos message to a tinyrobotics joint configuration vector
     * @param servos Servos message to convert
     * @tparam Servos type of the Servos message
     * @return tinyrobotics joint configuration vector
     */
    template <typename Servos, typename Scalar>
    inline Eigen::Matrix<Scalar, 20, 1> servos_to_configuration(const Servos* servos) {
        Eigen::Matrix<Scalar, 20, 1> q = Eigen::Matrix<Scalar, 20, 1>::Zero();
        for (const auto& [index, servo_id] : joint_map) {
            if (servo_exists(servos->servos, servo_id)) {
                q(index, 0) = servos->servos.at(servo_id).position;
            }
        }
        return q;
    }

    /**
     * @brief Converts a tinyrobotics joint configuration vector to a Servos message
     * @param servos Servos message to convert
     * @tparam Servos type of the Servos message
     * @return tinyrobotics joint configuration vector
     */
    template <typename Servos, typename Scalar>
    inline void configuration_to_servos(Servos* servos, const Eigen::Matrix<Scalar, 20, 1>& q) {
        for (const auto& [index, servo_id] : joint_map) {
            if (index < q.size() && servo_exists(servos->servos, servo_id)) {
                servos->servos[servo_id].position = q(index, 0);
            }
        }
    }

    /**
     * @brief Converts a Sensors message to a tinyrobotics joint configuration vector
     * @param sensors Sensors message to convert
     * @return tinyrobotics joint configuraSensorstion vector
     */
    template <typename Scalar>
    inline Eigen::Matrix<Scalar, 20, 1> sensors_to_configuration(const std::unique_ptr<Sensors>& sensors) {
        Eigen::Matrix<Scalar, 20, 1> q = Eigen::Matrix<Scalar, 20, 1>::Zero();
        for (const auto& [index, servo_id] : joint_map) {
            q(index, 0) = sensors->servo.at(servo_id).present_position;
        }
        return q;
    }

    /**
     * @brief Converts forward kinematics result to a map of ServoID to the associated forward kinematics transform
     * @param transforms vector of forward kinematics transforms with index corresponding to the tinyrobotics link
     index
     * @return map of ServoID to the associated forward kinematics transform
     */
    template <typename Scalar>
    inline std::map<ServoID, Eigen::Transform<Scalar, 3, Eigen::Isometry>> forward_kinematics_to_servo_map(
        const std::vector<Eigen::Transform<Scalar, 3, Eigen::Isometry>>& transforms) {
        std::map<ServoID, Eigen::Transform<Scalar, 3, Eigen::Isometry>> servo_map{};
        for (const auto& [link_index, servo_id] : link_map) {
            servo_map[servo_id] = transforms[link_index];
        }
        return servo_map;
    }

}  // namespace utility::actuation::tinyrobotics

#endif  // UTILITY_ACTUATION_TINYROBOTICS_HPP
