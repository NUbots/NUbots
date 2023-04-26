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

#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/angle.hpp"

namespace utility::actuation::tinyrobotics {

    using message::actuation::BodySide;
    using message::actuation::ServoCommand;
    using utility::input::LimbID;
    using utility::input::ServoID;

    /**
     * @brief Checks if a servo exists in a map of servos
     * @param servos_map map of servos
     * @param servo_id ID of the servo to check
     * @return true if the servo exists in the map
     */
    bool servo_exists(const std::map<u_int32_t, ServoCommand>& servos_map, ServoID servo_id) {
        return servos_map.find(servo_id) != servos_map.end();
    }

    /// @brief Maps the servo ID to the index in the tinyrobotics model joint configuration vector
    std::vector<std::pair<int, ServoID>> joint_map = {{11, ServoID::R_HIP_YAW},
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

    /** @brief Converts a Servos message to a tinyrobotics joint configuration vector
     * @param servos Servos message to convert
     * @tparam Servos type of the Servos message
     * @return tinyrobotics joint configuration vector
     */
    template <typename Servos>
    Eigen::Matrix<double, 20, 1> servos_to_configuration(const Servos* servos) {
        Eigen::Matrix<double, 20, 1> q = Eigen::Matrix<double, 20, 1>::Zero();
        for (const auto& [index, servo_id] : joint_map) {
            if (servo_exists(servos->servos, servo_id)) {
                q(index, 0) = servos->servos.at(servo_id).position;
            }
        }
        return q;
    }

    /** @brief Converts a tinyrobotics joint configuration vector to a Servos message
     * @param servos Servos message to convert
     * @tparam Servos type of the Servos message
     * @return tinyrobotics joint configuration vector
     */
    template <typename Servos>
    void configuration_to_servos(Servos* servos, const Eigen::Matrix<double, 20, 1>& q) {
        for (const auto& [index, servo_id] : joint_map) {
            if (index < q.size() && servo_exists(servos->servos, servo_id)) {
                servos->servos[servo_id].position = q(index, 0);
            }
        }
    }
}  // namespace utility::actuation::tinyrobotics

#endif  // UTILITY_ACTUATION_TINYROBOTICS_HPP
