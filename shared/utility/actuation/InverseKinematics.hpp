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
        @param target The target 4x4 basis matrix for the ankle
        @param isLeft Request for left leg motors or right leg motors?
        @param RobotKinematicsModel The class containing the leg model of the robot.
    */
    template <typename Scalar>
    [[nodiscard]] std::vector<std::pair<ServoID, Scalar>> calculateLegJoints(
        const message::actuation::KinematicsModel& model,
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& target_,
        const LimbID& limb) {
        const Scalar LENGTH_BETWEEN_LEGS             = model.leg.LENGTH_BETWEEN_LEGS;
        const Scalar DISTANCE_FROM_BODY_TO_HIP_JOINT = model.leg.HIP_OFFSET_Z;
        const Scalar HIP_OFFSET_X                    = model.leg.HIP_OFFSET_X;
        const Scalar UPPER_LEG_LENGTH                = model.leg.UPPER_LEG_LENGTH;
        const Scalar LOWER_LEG_LENGTH                = model.leg.LOWER_LEG_LENGTH;

        std::vector<std::pair<ServoID, Scalar>> positions;

        Scalar hipYaw     = 0;
        Scalar hipRoll    = 0;
        Scalar hipPitch   = 0;
        Scalar knee       = 0;
        Scalar anklePitch = 0;
        Scalar ankleRoll  = 0;

        // Correct for input referencing the bottom of the foot
        Eigen::Transform<Scalar, 3, Eigen::Isometry> target(target_);
        target = target.translate(Eigen::Matrix<Scalar, 3, 1>(0.0, 0.0, model.leg.FOOT_HEIGHT));

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
        Eigen::Matrix<Scalar, 3, 1> translation = (Tci * target.matrix().template rightCols<1>()).template head<3>();
        target                                  = Tci * target * Tci.transpose();
        target.translation()                    = translation;

        // swap legs if needed
        if (limb != LimbID::LEFT_LEG) {
            target(0, 1)             = -target(0, 1);
            target(0, 2)             = -target(0, 2);
            target(1, 0)             = -target(1, 0);
            target(2, 0)             = -target(2, 0);
            target.translation().x() = -target.translation().x();
        }

        const Eigen::Matrix<Scalar, 3, 1> ankleX   = target.matrix().template leftCols<1>().template head<3>();
        const Eigen::Matrix<Scalar, 3, 1> ankleY   = target.matrix().template middleCols<1>(1).template head<3>();
        const Eigen::Matrix<Scalar, 3, 1> anklePos = target.translation();

        const Eigen::Matrix<Scalar, 3, 1> hipOffset(LENGTH_BETWEEN_LEGS * 0.5,
                                                    HIP_OFFSET_X,
                                                    DISTANCE_FROM_BODY_TO_HIP_JOINT);

        Eigen::Matrix<Scalar, 3, 1> targetLeg = anklePos - hipOffset;

        const Scalar maxLegLength = UPPER_LEG_LENGTH + LOWER_LEG_LENGTH;
        Scalar length             = targetLeg.norm();
        if (length > maxLegLength) {
            targetLeg = targetLeg * maxLegLength / length;
            length    = targetLeg.norm();
        }
        const Scalar sqrLength   = length * length;
        const Scalar sqrUpperLeg = UPPER_LEG_LENGTH * UPPER_LEG_LENGTH;
        const Scalar sqrLowerLeg = LOWER_LEG_LENGTH * LOWER_LEG_LENGTH;

        const Scalar cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2.0 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH);
        knee                 = std::acos(std::max(std::min(cosKnee, Scalar(1.0)), Scalar(-1.0)));

        const Scalar cosLowerLeg = (sqrLowerLeg + sqrLength - sqrUpperLeg) / (2.0 * LOWER_LEG_LENGTH * length);
        const Scalar lowerLeg    = std::acos(std::max(std::min(cosLowerLeg, Scalar(1.0)), Scalar(-1.0)));

        const Scalar phi2 = std::acos(targetLeg.dot(ankleY) / length);

        anklePitch = lowerLeg + phi2 - M_PI_2;

        const Eigen::Matrix<Scalar, 3, 1> unitTargetLeg = targetLeg / length;

        Eigen::Matrix<Scalar, 3, 1> hipX = ankleY.cross(unitTargetLeg);
        Scalar hipXLength                = hipX.norm();
        if (hipXLength > 0) {
            hipX /= hipXLength;
        }
        else {
            NUClear::log<NUClear::DEBUG>(
                "InverseKinematics::calculateLegJoints : targetLeg and ankleY parallel. This is unhandled at "
                "the moment. requested pose = \n",
                target.matrix());
            return positions;
        }
        // Will be unit as ankleY and hipX are normal and unit
        const Eigen::Matrix<Scalar, 3, 1> legPlaneTangent = ankleY.cross(hipX);

        ankleRoll = std::atan2(ankleX.dot(legPlaneTangent), ankleX.dot(hipX));

        const bool isAnkleAboveWaist = unitTargetLeg.dot(Eigen::Matrix<Scalar, 3, 1>::UnitZ()) < 0;

        const Scalar cosZandHipX   = Eigen::Matrix<Scalar, 3, 1>::UnitZ().dot(hipX);
        const bool hipRollPositive = cosZandHipX <= 0;
        Eigen::Matrix<Scalar, 3, 1> legPlaneGlobalZ =
            (isAnkleAboveWaist ? -1 : 1) * (Eigen::Matrix<Scalar, 3, 1>::UnitZ() - (cosZandHipX * hipX));
        const Scalar legPlaneGlobalZLength = legPlaneGlobalZ.norm();
        if (legPlaneGlobalZLength > 0) {
            legPlaneGlobalZ /= legPlaneGlobalZLength;
        }

        const Scalar cosHipRoll = legPlaneGlobalZ.dot(Eigen::Matrix<Scalar, 3, 1>::UnitZ());
        hipRoll = (hipRollPositive ? 1 : -1) * std::acos(std::max(std::min(cosHipRoll, Scalar(1)), Scalar(-1)));

        const Scalar phi4 = M_PI - knee - lowerLeg;
        // Superposition values:
        const Scalar sinPIminusPhi2 = std::sin(M_PI - phi2);
        const Eigen::Matrix<Scalar, 3, 1> unitUpperLeg =
            unitTargetLeg * (std::sin(phi2 - phi4) / sinPIminusPhi2) + ankleY * (std::sin(phi4) / sinPIminusPhi2);
        const bool isHipPitchPositive = hipX.dot(unitUpperLeg.cross(legPlaneGlobalZ)) >= 0;

        hipPitch = (isHipPitchPositive ? 1 : -1)
                   * std::acos(std::max(std::min(legPlaneGlobalZ.dot(unitUpperLeg), Scalar(1)), Scalar(-1)));

        // If leg is above waist then hipX is pointing in the wrong direction in the xy plane
        Eigen::Matrix<Scalar, 3, 1> hipXProjected = (isAnkleAboveWaist ? -1 : 1) * hipX;
        hipXProjected.z()                         = 0;
        hipXProjected.normalize();
        const bool isHipYawPositive = hipXProjected.dot(Eigen::Matrix<Scalar, 3, 1>::UnitY()) >= 0;
        hipYaw                      = (isHipYawPositive ? 1 : -1)
                 * std::acos(std::max(std::min(hipXProjected.dot(Eigen::Matrix<Scalar, 3, 1>::UnitX()), Scalar(1)),
                                      Scalar(-1)));

        if (limb == LimbID::LEFT_LEG) {
            positions.push_back(std::make_pair(ServoID::L_HIP_YAW, -hipYaw));
            positions.push_back(std::make_pair(ServoID::L_HIP_ROLL, hipRoll));
            positions.push_back(std::make_pair(ServoID::L_HIP_PITCH, -hipPitch));
            positions.push_back(std::make_pair(ServoID::L_KNEE, M_PI - knee));
            positions.push_back(std::make_pair(ServoID::L_ANKLE_PITCH, -anklePitch));
            positions.push_back(std::make_pair(ServoID::L_ANKLE_ROLL, ankleRoll));
        }
        else {
            positions.push_back(std::make_pair(ServoID::R_HIP_YAW, (model.leg.LEFT_TO_RIGHT_HIP_YAW) * -hipYaw));
            positions.push_back(std::make_pair(ServoID::R_HIP_ROLL, (model.leg.LEFT_TO_RIGHT_HIP_ROLL) * hipRoll));
            positions.push_back(std::make_pair(ServoID::R_HIP_PITCH, (model.leg.LEFT_TO_RIGHT_HIP_PITCH) * -hipPitch));
            positions.push_back(std::make_pair(ServoID::R_KNEE, (model.leg.LEFT_TO_RIGHT_KNEE) * (M_PI - knee)));
            positions.push_back(
                std::make_pair(ServoID::R_ANKLE_PITCH, (model.leg.LEFT_TO_RIGHT_ANKLE_PITCH) * -anklePitch));
            positions.push_back(
                std::make_pair(ServoID::R_ANKLE_ROLL, (model.leg.LEFT_TO_RIGHT_ANKLE_ROLL) * ankleRoll));
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
    [[nodiscard]] std::vector<std::pair<ServoID, Scalar>> calculate_head_joints(
        const Eigen::Matrix<Scalar, 3, 1>& uPCt) {
        return {std::make_pair(ServoID::HEAD_YAW, std::atan2(uPCt.y(), uPCt.x())),
                std::make_pair(ServoID::HEAD_PITCH,
                               std::atan2(uPCt.z(), std::sqrt(uPCt.x() * uPCt.x() + uPCt.y() * uPCt.y())))};
    }
}  // namespace utility::actuation::kinematics

#endif  // UTILITY_ACTUATION_INVERSEKINEMATICS_HPP
