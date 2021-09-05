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

#include "InverseKinematics.hpp"

namespace utility::motion::kinematics {

    using message::input::Sensors;
    using message::motion::KinematicsModel;

    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;

    /*! @brief Calculates the leg joints for a given input ankle position.
            The robot coordinate system has origin a distance DISTANCE_FROM_BODY_TO_HIP_JOINT above the midpoint
       of the hips. Robot coordinate system: x is out of the front of the robot y is left, from right shoulder
       to left z is upward, from feet to head Input ankle coordinate system: x is forward, from heel to toe y is
       left, z is normal to the plane of the foot
        @param target The target 4x4 basis matrix for the ankle
        @param isLeft Request for left leg motors or right leg motors?
        @param RobotKinematicsModel The class containing the leg model of the robot.
    */
    std::vector<std::pair<ServoID, double>> calculateLegJoints(const KinematicsModel& model,
                                                               const Eigen::Affine3d& target_,
                                                               const LimbID& limb) {
        const double LENGTH_BETWEEN_LEGS             = model.leg.LENGTH_BETWEEN_LEGS;
        const double DISTANCE_FROM_BODY_TO_HIP_JOINT = model.leg.HIP_OFFSET_Z;
        const double HIP_OFFSET_X                    = model.leg.HIP_OFFSET_X;
        const double UPPER_LEG_LENGTH                = model.leg.UPPER_LEG_LENGTH;
        const double LOWER_LEG_LENGTH                = model.leg.LOWER_LEG_LENGTH;

        std::vector<std::pair<ServoID, double>> positions;

        double hipYaw     = 0;
        double hipRoll    = 0;
        double hipPitch   = 0;
        double knee       = 0;
        double anklePitch = 0;
        double ankleRoll  = 0;

        // Correct for input referencing the bottom of the foot
        Eigen::Affine3d target(target_);
        target = target.translate(Eigen::Vector3d(0.0, 0.0, model.leg.FOOT_HEIGHT));

        // Tci = transformation (not necessarily homogeneous) from input coordinates to calculation coordinates
        // TODO remove this. It was due to wrong convention use
        Eigen::Matrix4d Tci;
        // clang-format off
            Tci << 0.0, 1.0,  0.0, 0.0,
                   1.0, 0.0,  0.0, 0.0,
                   0.0, 0.0, -1.0, 0.0,
                   0.0, 0.0,  0.0, 1.0;
        // clang-format on
        // Rotate input position from standard robot coords to foot coords
        Eigen::Vector3d translation = (Tci * target.matrix().rightCols<1>()).head<3>();
        target                      = Tci * target * Tci.transpose();
        target.translation()        = translation;

        // swap legs if needed
        if (limb != LimbID::LEFT_LEG) {
            target(0, 1)             = -target(0, 1);
            target(0, 2)             = -target(0, 2);
            target(1, 0)             = -target(1, 0);
            target(2, 0)             = -target(2, 0);
            target.translation().x() = -target.translation().x();
        }

        const Eigen::Vector3d ankleX   = target.matrix().leftCols<1>().head<3>();
        const Eigen::Vector3d ankleY   = target.matrix().middleCols<1>(1).head<3>();
        const Eigen::Vector3d anklePos = target.translation();

        const Eigen::Vector3d hipOffset(LENGTH_BETWEEN_LEGS * 0.5, HIP_OFFSET_X, DISTANCE_FROM_BODY_TO_HIP_JOINT);

        Eigen::Vector3d targetLeg = anklePos - hipOffset;

        const double maxLegLength = UPPER_LEG_LENGTH + LOWER_LEG_LENGTH;
        double length             = targetLeg.norm();
        if (length > maxLegLength) {
            targetLeg = targetLeg * maxLegLength / length;
            length    = targetLeg.norm();
        }
        const double sqrLength   = length * length;
        const double sqrUpperLeg = UPPER_LEG_LENGTH * UPPER_LEG_LENGTH;
        const double sqrLowerLeg = LOWER_LEG_LENGTH * LOWER_LEG_LENGTH;

        const double cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2.0 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH);
        knee                 = std::acos(std::fmax(std::fmin(cosKnee, 1), -1));

        const double cosLowerLeg = (sqrLowerLeg + sqrLength - sqrUpperLeg) / (2.0 * LOWER_LEG_LENGTH * length);
        const double lowerLeg    = std::acos(std::fmax(std::fmin(cosLowerLeg, 1), -1));

        const double phi2 = std::acos(targetLeg.dot(ankleY) / length);

        anklePitch = lowerLeg + phi2 - M_PI_2;

        const Eigen::Vector3d unitTargetLeg = targetLeg / length;

        Eigen::Vector3d hipX = ankleY.cross(unitTargetLeg);
        double hipXLength    = hipX.norm();
        if (hipXLength > 0) {
            hipX /= hipXLength;
        }
        else {
            NUClear::log<NUClear::DEBUG>(
                "InverseKinematics::calculateLegJoints : targetLeg and ankleY parallel. This is unhandled at "
                "the "
                "moment. requested pose = \n",
                target.matrix());
            return positions;
        }
        // Will be unit as ankleY and hipX are normal and unit
        const Eigen::Vector3d legPlaneTangent = ankleY.cross(hipX);

        ankleRoll = std::atan2(ankleX.dot(legPlaneTangent), ankleX.dot(hipX));

        const bool isAnkleAboveWaist = unitTargetLeg.dot(Eigen::Vector3d::UnitZ()) < 0;

        const double cosZandHipX   = Eigen::Vector3d::UnitZ().dot(hipX);
        const bool hipRollPositive = cosZandHipX <= 0;
        Eigen::Vector3d legPlaneGlobalZ =
            (isAnkleAboveWaist ? -1 : 1) * (Eigen::Vector3d::UnitZ() - (cosZandHipX * hipX));
        double legPlaneGlobalZLength = legPlaneGlobalZ.norm();
        if (legPlaneGlobalZLength > 0) {
            legPlaneGlobalZ /= legPlaneGlobalZLength;
        }

        const double cosHipRoll = legPlaneGlobalZ.dot(Eigen::Vector3d::UnitZ());
        hipRoll                 = (hipRollPositive ? 1 : -1) * std::acos(std::fmax(std::fmin(cosHipRoll, 1), -1));

        const double phi4 = M_PI - knee - lowerLeg;
        // Superposition values:
        const float sinPIminusPhi2 = std::sin(M_PI - phi2);
        const Eigen::Vector3d unitUpperLeg =
            unitTargetLeg * (std::sin(phi2 - phi4) / sinPIminusPhi2) + ankleY * (std::sin(phi4) / sinPIminusPhi2);
        const bool isHipPitchPositive = hipX.dot(unitUpperLeg.cross(legPlaneGlobalZ)) >= 0;

        hipPitch =
            (isHipPitchPositive ? 1 : -1) * std::acos(std::fmax(std::fmin(legPlaneGlobalZ.dot(unitUpperLeg), 1), -1));

        // If leg is above waist then hipX is pointing in the wrong direction in the xy plane
        Eigen::Vector3d hipXProjected = (isAnkleAboveWaist ? -1 : 1) * hipX;
        hipXProjected.z()             = 0;
        hipXProjected.normalize();
        const bool isHipYawPositive = hipXProjected.dot(Eigen::Vector3d::UnitY()) >= 0;
        hipYaw                      = (isHipYawPositive ? 1 : -1)
                 * std::acos(std::fmax(std::fmin(hipXProjected.dot(Eigen::Vector3d::UnitX()), 1), -1));

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

    std::vector<std::pair<ServoID, float>> calculateLegJoints(const KinematicsModel& model,
                                                              const Eigen::Affine3f& target_,
                                                              const LimbID& limb) {
        const float LENGTH_BETWEEN_LEGS             = model.leg.LENGTH_BETWEEN_LEGS;
        const float DISTANCE_FROM_BODY_TO_HIP_JOINT = model.leg.HIP_OFFSET_Z;
        const float HIP_OFFSET_X                    = model.leg.HIP_OFFSET_X;
        const float UPPER_LEG_LENGTH                = model.leg.UPPER_LEG_LENGTH;
        const float LOWER_LEG_LENGTH                = model.leg.LOWER_LEG_LENGTH;

        std::vector<std::pair<ServoID, float>> positions;

        float hipYaw     = 0;
        float hipRoll    = 0;
        float hipPitch   = 0;
        float knee       = 0;
        float anklePitch = 0;
        float ankleRoll  = 0;

        // Correct for input referencing the bottom of the foot
        Eigen::Affine3f target(target_);
        target = target.translate(Eigen::Vector3f(0.0, 0.0, model.leg.FOOT_HEIGHT));

        // Tci = transformation (not necessarily homogeneous) from input coordinates to calculation coordinates
        // TODO remove this. It was due to wrong convention use
        Eigen::Matrix4f Tci;
        // clang-format off
            Tci << 0.0, 1.0,  0.0, 0.0,
                   1.0, 0.0,  0.0, 0.0,
                   0.0, 0.0, -1.0, 0.0,
                   0.0, 0.0,  0.0, 1.0;
        // clang-format on
        // Rotate input position from standard robot coords to foot coords
        Eigen::Vector3f translation = (Tci * target.matrix().rightCols<1>()).head<3>();
        target                      = Tci * target * Tci.transpose();
        target.translation()        = translation;

        // swap legs if needed
        if (limb != LimbID::LEFT_LEG) {
            target(0, 1)             = -target(0, 1);
            target(0, 2)             = -target(0, 2);
            target(1, 0)             = -target(1, 0);
            target(2, 0)             = -target(2, 0);
            target.translation().x() = -target.translation().x();
        }

        const Eigen::Vector3f ankleX   = target.matrix().leftCols<1>().head<3>();
        const Eigen::Vector3f ankleY   = target.matrix().middleCols<1>(1).head<3>();
        const Eigen::Vector3f anklePos = target.translation();

        const Eigen::Vector3f hipOffset(LENGTH_BETWEEN_LEGS * 0.5, HIP_OFFSET_X, DISTANCE_FROM_BODY_TO_HIP_JOINT);

        Eigen::Vector3f targetLeg = anklePos - hipOffset;

        const float maxLegLength = UPPER_LEG_LENGTH + LOWER_LEG_LENGTH;
        float length             = targetLeg.norm();
        if (length > maxLegLength) {
            targetLeg = targetLeg * maxLegLength / length;
            length    = targetLeg.norm();
        }
        const float sqrLength   = length * length;
        const float sqrUpperLeg = UPPER_LEG_LENGTH * UPPER_LEG_LENGTH;
        const float sqrLowerLeg = LOWER_LEG_LENGTH * LOWER_LEG_LENGTH;

        const float cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2.0 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH);
        knee                = std::acos(std::fmax(std::fmin(cosKnee, 1), -1));

        const float cosLowerLeg = (sqrLowerLeg + sqrLength - sqrUpperLeg) / (2.0 * LOWER_LEG_LENGTH * length);
        const float lowerLeg    = std::acos(std::fmax(std::fmin(cosLowerLeg, 1), -1));

        const float phi2 = std::acos(targetLeg.dot(ankleY) / length);

        anklePitch = lowerLeg + phi2 - M_PI_2;

        const Eigen::Vector3f unitTargetLeg = targetLeg / length;

        Eigen::Vector3f hipX = ankleY.cross(unitTargetLeg);
        float hipXLength     = hipX.norm();
        if (hipXLength > 0) {
            hipX /= hipXLength;
        }
        else {
            NUClear::log<NUClear::DEBUG>(
                "InverseKinematics::calculateLegJoints : targetLeg and ankleY parallel. This is unhandled at "
                "the "
                "moment. requested pose = \n",
                target.matrix());
            return positions;
        }
        // Will be unit as ankleY and hipX are normal and unit
        const Eigen::Vector3f legPlaneTangent = ankleY.cross(hipX);

        ankleRoll = std::atan2(ankleX.dot(legPlaneTangent), ankleX.dot(hipX));

        const bool isAnkleAboveWaist = unitTargetLeg.dot(Eigen::Vector3f::UnitZ()) < 0;

        const float cosZandHipX    = Eigen::Vector3f::UnitZ().dot(hipX);
        const bool hipRollPositive = cosZandHipX <= 0;
        Eigen::Vector3f legPlaneGlobalZ =
            (isAnkleAboveWaist ? -1 : 1) * (Eigen::Vector3f::UnitZ() - (cosZandHipX * hipX));
        const float legPlaneGlobalZLength = legPlaneGlobalZ.norm();
        if (legPlaneGlobalZLength > 0) {
            legPlaneGlobalZ /= legPlaneGlobalZLength;
        }

        const float cosHipRoll = legPlaneGlobalZ.dot(Eigen::Vector3f::UnitZ());
        hipRoll                = (hipRollPositive ? 1 : -1) * std::acos(std::fmax(std::fmin(cosHipRoll, 1), -1));

        const float phi4 = M_PI - knee - lowerLeg;
        // Superposition values:
        const float sinPIminusPhi2 = std::sin(M_PI - phi2);
        const Eigen::Vector3f unitUpperLeg =
            unitTargetLeg * (std::sin(phi2 - phi4) / sinPIminusPhi2) + ankleY * (std::sin(phi4) / sinPIminusPhi2);
        const bool isHipPitchPositive = hipX.dot(unitUpperLeg.cross(legPlaneGlobalZ)) >= 0;

        hipPitch =
            (isHipPitchPositive ? 1 : -1) * std::acos(std::fmax(std::fmin(legPlaneGlobalZ.dot(unitUpperLeg), 1), -1));

        // If leg is above waist then hipX is pointing in the wrong direction in the xy plane
        Eigen::Vector3f hipXProjected = (isAnkleAboveWaist ? -1 : 1) * hipX;
        hipXProjected.z()             = 0;
        hipXProjected.normalize();
        const bool isHipYawPositive = hipXProjected.dot(Eigen::Vector3f::UnitY()) >= 0;
        hipYaw                      = (isHipYawPositive ? 1 : -1)
                 * std::acos(std::fmax(std::fmin(hipXProjected.dot(Eigen::Vector3f::UnitX()), 1), -1));

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
    std::vector<std::pair<ServoID, float>> calculateLegJoints(const message::motion::KinematicsModel& model,
                                                              const Eigen::Affine3f& leftTarget,
                                                              const Eigen::Affine3f& rightTarget) {
        auto joints  = calculateLegJoints(model, leftTarget, LimbID::LEFT_LEG);
        auto joints2 = calculateLegJoints(model, rightTarget, LimbID::RIGHT_LEG);
        joints.insert(joints.end(), joints2.begin(), joints2.end());
        return joints;
    }
    std::vector<std::pair<ServoID, double>> calculateCameraLookJoints(const Eigen::Vector3d& cameraUnitVector) {
        std::vector<std::pair<ServoID, double>> positions;
        positions.push_back(std::make_pair(ServoID::HEAD_YAW, std::atan2(cameraUnitVector.y(), cameraUnitVector.x())));
        positions.push_back(std::make_pair(ServoID::HEAD_PITCH,
                                           std::atan2(-cameraUnitVector.z(),
                                                      std::sqrt(cameraUnitVector.x() * cameraUnitVector.x()
                                                                + cameraUnitVector.y() * cameraUnitVector.y()))));
        return positions;
    }
    std::vector<std::pair<ServoID, float>> calculateHeadJoints(Eigen::Vector3f cameraUnitVector) {
        std::vector<std::pair<ServoID, float>> positions;
        positions.push_back(std::make_pair(ServoID::HEAD_YAW, atan2(cameraUnitVector[1], cameraUnitVector[0])));
        positions.push_back(std::make_pair(
            ServoID::HEAD_PITCH,
            atan2(-cameraUnitVector[2],
                  std::sqrt(cameraUnitVector[0] * cameraUnitVector[0] + cameraUnitVector[1] * cameraUnitVector[1]))));
        return positions;
    }

}  // namespace utility::motion::kinematics
