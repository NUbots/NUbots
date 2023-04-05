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

#ifndef UTILITY_ACTUATION_INVERSEKINEMATICS_HPP
#define UTILITY_ACTUATION_INVERSEKINEMATICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <limits>
#include <nuclear>
#include <vector>

#include "message/actuation/KinematicsModel.hpp"
#include "message/input/Sensors.hpp"

#include "utility/actuation/ForwardKinematics.hpp"
#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/coordinates.hpp"

namespace utility::actuation::kinematics {

    using utility::input::LimbID;
    using utility::input::ServoID;

    /*! @brief Calculates the leg joints for a given input ankle position.
            The robot coordinate system has origin a distance DISTANCE_FROM_BODY_TO_HIP_JOINT above the midpoint
       of the hips. Robot coordinate system: x is out of the front of the robot y is left, from right shoulder
       to left z is upward, from feet to head Input ankle coordinate system: x is forward, from heel to toe y is
       left, z is normal to the plane of the foot
        @param Htf The target transform of the foot from the torso.
        @param isLeft Request for left leg motors or right leg motors?
        @param RobotKinematicsModel The class containing the leg model of the robot.
    */
    template <typename Scalar>
    [[nodiscard]] std::vector<std::pair<ServoID, Scalar>> calculateLegJoints(
        const message::actuation::KinematicsModel& model,
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Htf,
        const LimbID& limb) {
        const Scalar LENGTH_BETWEEN_LEGS             = model.leg.LENGTH_BETWEEN_LEGS;
        const Scalar DISTANCE_FROM_BODY_TO_HIP_JOINT = model.leg.HIP_OFFSET_Z;
        const Scalar HIP_OFFSET_X                    = model.leg.HIP_OFFSET_X;
        const Scalar UPPER_LEG_LENGTH                = model.leg.UPPER_LEG_LENGTH;
        const Scalar LOWER_LEG_LENGTH                = model.leg.LOWER_LEG_LENGTH;

        std::vector<std::pair<ServoID, Scalar>> positions;

        Scalar hip_yaw     = 0;
        Scalar hip_roll    = 0;
        Scalar hip_pitch   = 0;
        Scalar knee        = 0;
        Scalar ankle_pitch = 0;
        Scalar ankle_roll  = 0;

        // Correct for input referencing the bottom of the foot
        Htf = Htf.translate(Eigen::Matrix<Scalar, 3, 1>(0.0, 0.0, model.leg.FOOT_HEIGHT));
        // Tci = transformation (not necessarily homogeneous) from input coordinates to calculation coordinates
        // TODO remove this. It was due to wrong convention use
        Eigen::Matrix<Scalar, 4, 4> Tci;
        // clang-format off
            Tci << 0.0, 1.0,  0.0, 0.0,
                   1.0, 0.0,  0.0, 0.0,
                   0.0, 0.0, -1.0, 0.0,
                   0.0, 0.0,  0.0, 1.0;
        // clang-format on
        // Rotate input position from standard robot coords to foot coords
        Eigen::Matrix<Scalar, 3, 1> translation = (Tci * Htf.matrix().template rightCols<1>()).template head<3>();
        Htf                                     = Tci * Htf * Tci.transpose();
        Htf.translation()                       = translation;

        // Swap legs if needed
        if (limb != LimbID::LEFT_LEG) {
            Htf(0, 1)             = -Htf(0, 1);
            Htf(0, 2)             = -Htf(0, 2);
            Htf(1, 0)             = -Htf(1, 0);
            Htf(2, 0)             = -Htf(2, 0);
            Htf.translation().x() = -Htf.translation().x();
        }

        const Eigen::Matrix<Scalar, 3, 1> ankle_x   = Htf.matrix().template leftCols<1>().template head<3>();
        const Eigen::Matrix<Scalar, 3, 1> ankle_y   = Htf.matrix().template middleCols<1>(1).template head<3>();
        const Eigen::Matrix<Scalar, 3, 1> ankle_pos = Htf.translation();

        const Eigen::Matrix<Scalar, 3, 1> hip_offset(LENGTH_BETWEEN_LEGS * 0.5,
                                                     HIP_OFFSET_X,
                                                     -DISTANCE_FROM_BODY_TO_HIP_JOINT);

        Eigen::Matrix<Scalar, 3, 1> Htf_leg = ankle_pos - hip_offset;

        const Scalar max_leg_length = UPPER_LEG_LENGTH + LOWER_LEG_LENGTH;
        Scalar length               = Htf_leg.norm();
        if (length > max_leg_length) {
            Htf_leg = Htf_leg * max_leg_length / length;
            length  = Htf_leg.norm();
        }
        const Scalar sqr_length    = length * length;
        const Scalar sqr_upperLeg  = UPPER_LEG_LENGTH * UPPER_LEG_LENGTH;
        const Scalar sqr_lower_leg = LOWER_LEG_LENGTH * LOWER_LEG_LENGTH;

        const Scalar cosKnee =
            (sqr_upperLeg + sqr_lower_leg - sqr_length) / (2.0 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH);
        knee = std::acos(std::max(std::min(cosKnee, Scalar(1.0)), Scalar(-1.0)));

        const Scalar cos_lower_leg = (sqr_lower_leg + sqr_length - sqr_upperLeg) / (2.0 * LOWER_LEG_LENGTH * length);
        const Scalar lower_leg     = std::acos(std::max(std::min(cos_lower_leg, Scalar(1.0)), Scalar(-1.0)));

        const Scalar phi2 = std::acos(Htf_leg.dot(ankle_y) / length);

        ankle_pitch = lower_leg + phi2 - M_PI_2;

        const Eigen::Matrix<Scalar, 3, 1> unit_target_leg = Htf_leg / length;

        Eigen::Matrix<Scalar, 3, 1> hip_x = ankle_y.cross(unit_target_leg);
        Scalar hip_xLength                = hip_x.norm();
        if (hip_xLength > 0) {
            hip_x /= hip_xLength;
        }
        else {
            NUClear::log<NUClear::DEBUG>(
                "InverseKinematics::calculateLegJoints : Htf_leg and ankle_y parallel. This is unhandled at "
                "the moment. requested pose = \n",
                Htf.matrix());
            return positions;
        }
        // Will be unit as ankle_y and hip_x are normal and unit
        const Eigen::Matrix<Scalar, 3, 1> leg_plane_tangent = ankle_y.cross(hip_x);

        ankle_roll = std::atan2(ankle_x.dot(leg_plane_tangent), ankle_x.dot(hip_x));

        const bool is_ankle_above_waist = unit_target_leg.dot(Eigen::Matrix<Scalar, 3, 1>::UnitZ()) < 0;

        const Scalar cos_z_and_hip_x = Eigen::Matrix<Scalar, 3, 1>::UnitZ().dot(hip_x);
        const bool hip_roll_positive = cos_z_and_hip_x <= 0;
        Eigen::Matrix<Scalar, 3, 1> leg_plane_global_z =
            (is_ankle_above_waist ? -1 : 1) * (Eigen::Matrix<Scalar, 3, 1>::UnitZ() - (cos_z_and_hip_x * hip_x));
        const Scalar leg_plane_global_z_length = leg_plane_global_z.norm();
        if (leg_plane_global_z_length > 0) {
            leg_plane_global_z /= leg_plane_global_z_length;
        }

        const Scalar cos_hip_roll = leg_plane_global_z.dot(Eigen::Matrix<Scalar, 3, 1>::UnitZ());
        hip_roll = (hip_roll_positive ? 1 : -1) * std::acos(std::max(std::min(cos_hip_roll, Scalar(1)), Scalar(-1)));

        const Scalar phi4 = M_PI - knee - lower_leg;
        // Superposition values:
        const Scalar sin_PI_minus_phi_2 = std::sin(M_PI - phi2);
        const Eigen::Matrix<Scalar, 3, 1> unit_upper_leg =
            unit_target_leg * (std::sin(phi2 - phi4) / sin_PI_minus_phi_2)
            + ankle_y * (std::sin(phi4) / sin_PI_minus_phi_2);
        const bool is_hip_pitch_positive = hip_x.dot(unit_upper_leg.cross(leg_plane_global_z)) >= 0;

        hip_pitch = (is_hip_pitch_positive ? 1 : -1)
                    * std::acos(std::max(std::min(leg_plane_global_z.dot(unit_upper_leg), Scalar(1)), Scalar(-1)));

        // If leg is above waist then hip_x is pointing in the wrong direction in the xy plane
        Eigen::Matrix<Scalar, 3, 1> hip_x_projected = (is_ankle_above_waist ? -1 : 1) * hip_x;
        hip_x_projected.z()                         = 0;
        hip_x_projected.normalize();
        const bool is_hip_yaw_positive = hip_x_projected.dot(Eigen::Matrix<Scalar, 3, 1>::UnitY()) >= 0;
        hip_yaw                        = (is_hip_yaw_positive ? 1 : -1)
                  * std::acos(std::max(std::min(hip_x_projected.dot(Eigen::Matrix<Scalar, 3, 1>::UnitX()), Scalar(1)),
                                       Scalar(-1)));

        if (limb == LimbID::LEFT_LEG) {
            positions.push_back(std::make_pair(ServoID::L_HIP_YAW, -hip_yaw));
            positions.push_back(std::make_pair(ServoID::L_HIP_ROLL, hip_roll));
            positions.push_back(std::make_pair(ServoID::L_HIP_PITCH, -hip_pitch));
            positions.push_back(std::make_pair(ServoID::L_KNEE, M_PI - knee));
            positions.push_back(std::make_pair(ServoID::L_ANKLE_PITCH, -ankle_pitch));
            positions.push_back(std::make_pair(ServoID::L_ANKLE_ROLL, ankle_roll));
        }
        else {
            positions.push_back(std::make_pair(ServoID::R_HIP_YAW, (model.leg.LEFT_TO_RIGHT_HIP_YAW) * -hip_yaw));
            positions.push_back(std::make_pair(ServoID::R_HIP_ROLL, (model.leg.LEFT_TO_RIGHT_HIP_ROLL) * hip_roll));
            positions.push_back(std::make_pair(ServoID::R_HIP_PITCH, (model.leg.LEFT_TO_RIGHT_HIP_PITCH) * -hip_pitch));
            positions.push_back(std::make_pair(ServoID::R_KNEE, (model.leg.LEFT_TO_RIGHT_KNEE) * (M_PI - knee)));
            positions.push_back(
                std::make_pair(ServoID::R_ANKLE_PITCH, (model.leg.LEFT_TO_RIGHT_ANKLE_PITCH) * -ankle_pitch));
            positions.push_back(
                std::make_pair(ServoID::R_ANKLE_ROLL, (model.leg.LEFT_TO_RIGHT_ANKLE_ROLL) * ankle_roll));
        }

        return positions;
    }

    template <typename Scalar>
    [[nodiscard]] std::vector<std::pair<ServoID, Scalar>> calculateLegJoints(
        const message::actuation::KinematicsModel& model,
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& leftTarget,
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& rightTarget) {
        auto joints  = calculateLegJoints<Scalar>(model, leftTarget, LimbID::LEFT_LEG);
        auto joints2 = calculateLegJoints<Scalar>(model, rightTarget, LimbID::RIGHT_LEG);
        joints.insert(joints.end(), joints2.begin(), joints2.end());
        return joints;
    }

    template <typename Scalar>
    [[nodiscard]] std::vector<std::pair<ServoID, Scalar>> calculateHeadJoints(
        const Eigen::Matrix<Scalar, 3, 1>& cameraUnitVector) {
        return {std::make_pair(ServoID::HEAD_YAW, std::atan2(cameraUnitVector.y(), cameraUnitVector.x())),
                std::make_pair(ServoID::HEAD_PITCH,
                               std::atan2(-cameraUnitVector.z(),
                                          std::sqrt(cameraUnitVector.x() * cameraUnitVector.x()
                                                    + cameraUnitVector.y() * cameraUnitVector.y())))};
    }
}  // namespace utility::actuation::kinematics

#endif  // UTILITY_ACTUATION_INVERSEKINEMATICS_HPP
