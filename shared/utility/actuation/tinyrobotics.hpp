/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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

#include "utility/input/FrameID.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/angle.hpp"

namespace utility::actuation::tinyrobotics {

    using message::actuation::BodySide;
    using message::actuation::ServoCommand;
    using utility::input::FrameID;
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
    /// @brief Maps the tinyrobotics joint index to ServoID for the K1 robot.
    /// Indices match the K1 URDF joint order as reported by tinyrobotics::Model::show_details().
    inline std::vector<std::pair<int, ServoID>> joint_map =
                                                     {{0,  ServoID::HEAD_YAW},
                                                      {1,  ServoID::HEAD_PITCH},
                                                      {2,  ServoID::L_SHOULDER_PITCH},
                                                      {3,  ServoID::L_SHOULDER_ROLL},
                                                      {4,  ServoID::L_ELBOW},
                                                      {5,  ServoID::L_ELBOW_YAW},
                                                      {6,  ServoID::R_SHOULDER_PITCH},
                                                      {7,  ServoID::R_SHOULDER_ROLL},
                                                      {8,  ServoID::R_ELBOW},
                                                      {9,  ServoID::R_ELBOW_YAW},
                                                      {10, ServoID::L_HIP_PITCH},
                                                      {11, ServoID::L_HIP_ROLL},
                                                      {12, ServoID::L_HIP_YAW},
                                                      {13, ServoID::L_KNEE},
                                                      {14, ServoID::L_ANKLE_PITCH},
                                                      {15, ServoID::L_ANKLE_ROLL},
                                                      {16, ServoID::R_HIP_PITCH},
                                                      {17, ServoID::R_HIP_ROLL},
                                                      {18, ServoID::R_HIP_YAW},
                                                      {19, ServoID::R_KNEE},
                                                      {20, ServoID::R_ANKLE_PITCH},
                                                      {21, ServoID::R_ANKLE_ROLL}};

    /// @brief Maps the tinyrobotics link index to FrameID for the K1 robot.
    /// Link indices match the K1 URDF as reported by tinyrobotics::Model::show_details().
    inline std::vector<std::pair<int, FrameID>> link_map = {
                                                      {1,  FrameID::HEAD_YAW},
                                                      {2,  FrameID::HEAD_PITCH},
                                                      {3,  FrameID::L_SHOULDER_PITCH},
                                                      {4,  FrameID::L_SHOULDER_ROLL},
                                                      {5,  FrameID::L_ELBOW},
                                                      {7,  FrameID::R_SHOULDER_PITCH},
                                                      {8,  FrameID::R_SHOULDER_ROLL},
                                                      {9,  FrameID::R_ELBOW},
                                                      {11, FrameID::L_HIP_PITCH},
                                                      {12, FrameID::L_HIP_ROLL},
                                                      {13, FrameID::L_HIP_YAW},
                                                      {14, FrameID::L_KNEE},
                                                      {15, FrameID::L_ANKLE_PITCH},
                                                      {16, FrameID::L_FOOT_BASE},
                                                      {17, FrameID::R_HIP_PITCH},
                                                      {18, FrameID::R_HIP_ROLL},
                                                      {19, FrameID::R_HIP_YAW},
                                                      {20, FrameID::R_KNEE},
                                                      {21, FrameID::R_ANKLE_PITCH},
                                                      {22, FrameID::R_FOOT_BASE}};
    // clang-format on

    /**
     * @brief Converts a Servos message to a tinyrobotics joint configuration vector
     * @param servos Servos message to convert
     * @tparam Servos type of the Servos message
     * @return tinyrobotics joint configuration vector
     */
    template <typename Servos, typename Scalar, int nq>
    inline Eigen::Matrix<Scalar, nq, 1> servos_to_configuration(const Servos* servos) {
        Eigen::Matrix<Scalar, nq, 1> q = Eigen::Matrix<Scalar, nq, 1>::Zero();
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
    template <typename Servos, typename Scalar, int nq>
    inline void configuration_to_servos(Servos* servos, const Eigen::Matrix<Scalar, nq, 1>& q) {
        for (const auto& [index, servo_id] : joint_map) {
            if (index < q.size() && servo_exists(servos->servos, servo_id)) {
                servos->servos[servo_id].position = q(index, 0);
            }
        }
    }

    /**
     * @brief Converts a Sensors message to a tinyrobotics joint configuration vector
     * @param sensors Sensors message to convert
     * @return tinyrobotics joint configuration vector
     */
    template <typename Scalar, int nq>
    inline Eigen::Matrix<Scalar, Eigen::Dynamic, 1> sensors_to_configuration(const std::unique_ptr<Sensors>& sensors) {
        Eigen::Matrix<Scalar, nq, 1> q = Eigen::Matrix<Scalar, nq, 1>::Zero();
        q.resize(sensors->servo.size(), 1);
        for (const auto& [index, servo_id] : joint_map) {
            q(index, 0) = sensors->servo.at(servo_id).present_position;
        }
        return q;
    }

    /**
     * @brief Converts forward kinematics result to a map of FrameID to the associated forward kinematics transform
     * @param transforms vector of forward kinematics transforms with index corresponding to the tinyrobotics link index
     * @return map of FrameID to the associated forward kinematics transform
     */
    template <typename Scalar>
    inline std::map<FrameID, Eigen::Transform<Scalar, 3, Eigen::Isometry>> forward_kinematics_to_servo_map(
        const std::vector<Eigen::Transform<Scalar, 3, Eigen::Isometry>>& transforms) {
        std::map<FrameID, Eigen::Transform<Scalar, 3, Eigen::Isometry>> servo_map{};
        for (const auto& [link_index, servo_id] : link_map) {
            servo_map[servo_id] = transforms[link_index];
        }
        return servo_map;
    }

}  // namespace utility::actuation::tinyrobotics

#endif  // UTILITY_ACTUATION_TINYROBOTICS_HPP
