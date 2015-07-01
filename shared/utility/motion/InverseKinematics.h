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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_MOTION_INVERSEKINEMATICS_H
#define UTILITY_MOTION_INVERSEKINEMATICS_H

#include <vector>
#include <armadillo>
#include <nuclear_bits/LogLevel.h>
#include <cmath>
#include <nuclear>

#include "utility/math/matrix/Transform3D.h"
#include "utility/math/coordinates.h"
#include "utility/motion/RobotModels.h"
#include "messages/input/ServoID.h"
#include "messages/input/Sensors.h"
#include "messages/behaviour/Action.h"

namespace utility {
namespace motion {
namespace kinematics {

    /*! @brief Calculates the leg joints for a given input ankle position.
            The robot coordinate system has origin a distance DISTANCE_FROM_BODY_TO_HIP_JOINT above the midpoint of the hips.
            Robot coordinate system:
                        x is out of the front of the robot
                        y is left, from right shoulder to left
                        z is upward, from feet to head
            Input ankle coordinate system:
                        x is forward, from heel to toe
                        y is left,
                        z is normal to the plane of the foot
        @param target The target 4x4 basis matrix for the ankle
        @param isLeft Request for left leg motors or right leg motors?
        @param RobotKinematicModel The class containing the leg model of the robot.
    */
    template <typename RobotKinematicModel>
    std::vector<std::pair<messages::input::ServoID, float>> calculateLegJoints(utility::math::matrix::Transform3D target, messages::input::LimbID limb) {
        const float LENGTH_BETWEEN_LEGS = RobotKinematicModel::Leg::LENGTH_BETWEEN_LEGS;
        const float DISTANCE_FROM_BODY_TO_HIP_JOINT = RobotKinematicModel::Leg::HIP_OFFSET_Z;
        const float HIP_OFFSET_X = RobotKinematicModel::Leg::HIP_OFFSET_X;
        const float UPPER_LEG_LENGTH = RobotKinematicModel::Leg::UPPER_LEG_LENGTH;
        const float LOWER_LEG_LENGTH = RobotKinematicModel::Leg::LOWER_LEG_LENGTH;

        std::vector<std::pair<messages::input::ServoID, float> > positions;

        float hipYaw = 0;
        float hipRoll = 0;
        float hipPitch = 0;
        float knee = 0;
        float anklePitch = 0;
        float ankleRoll = 0;

        //TODO remove this. It was due to wrong convention use
        utility::math::matrix::Transform3D inputCoordinatesToCalcCoordinates;
        inputCoordinatesToCalcCoordinates << 0<< 1<< 0<< 0<< arma::endr
                                          << 1<< 0<< 0<< 0<< arma::endr
                                          << 0<< 0<<-1<< 0<< arma::endr
                                          << 0<< 0<< 0<< 1;
        //Rotate input position from standard robot coords to foot coords
        //NUClear::log<NUClear::DEBUG>("Target Original\n", target);
        arma::vec4 fourthColumn = inputCoordinatesToCalcCoordinates * target.col(3);
        target = inputCoordinatesToCalcCoordinates * target * inputCoordinatesToCalcCoordinates.t();
        target.col(3) = fourthColumn;
        //NUClear::log<NUClear::DEBUG>("Target Final\n", target);

        if (limb != messages::input::LimbID::LEFT_LEG) {
            target.submat(0,0,2,2) = arma::mat33{-1,0,0, 0,1,0, 0,0,1} * target.submat(0,0,2,2);
            target.submat(0,0,2,0) *= -1;
            target(0,3) *= -1;
        }

        arma::vec3 ankleX = target.submat(0,0,2,0);
        arma::vec3 ankleY = target.submat(0,1,2,1);
        arma::vec3 ankleZ = target.submat(0,2,2,2);

        arma::vec3 anklePos = target.submat(0,3,2,3);

        arma::vec3 hipOffset = {LENGTH_BETWEEN_LEGS / 2.0, HIP_OFFSET_X, DISTANCE_FROM_BODY_TO_HIP_JOINT};

        arma::vec3 targetLeg = anklePos - hipOffset;

        float length = arma::norm(targetLeg, 2);
        if (length > UPPER_LEG_LENGTH+LOWER_LEG_LENGTH){
            NUClear::log<NUClear::WARN>("InverseKinematics::calculateLegJoints : !!! WARNING !!! Requested position beyond leg reach. Scaling back requested vector.");
            targetLeg *= (UPPER_LEG_LENGTH+LOWER_LEG_LENGTH)/length;
            length = UPPER_LEG_LENGTH+LOWER_LEG_LENGTH;
        }
        ////NUClear::log<NUClear::DEBUG>("Length: ", length);
        float sqrLength = length * length;
        float sqrUpperLeg = UPPER_LEG_LENGTH * UPPER_LEG_LENGTH;
        float sqrLowerLeg = LOWER_LEG_LENGTH * LOWER_LEG_LENGTH;

        float cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH);
       // //NUClear::log<NUClear::DEBUG>("Cos Knee: ", cosKnee);
        // TODO: check if cosKnee is between 1 and -1
        knee = acos(cosKnee);

        float cosLowerLeg = (sqrLowerLeg + sqrLength - sqrUpperLeg) / (2 * LOWER_LEG_LENGTH * length);
        // TODO: check if cosLowerLeg is between 1 and -1
        float lowerLeg = acos(cosLowerLeg);

        float phi2 = acos(arma::dot(targetLeg, ankleY)/length);

        anklePitch = lowerLeg + phi2 - M_PI_2;

        arma::vec3 unitTargetLeg = targetLeg / length;

        arma::vec3 hipX = arma::cross(ankleY, unitTargetLeg);
        float hipXLength = arma::norm(hipX,2);
        if (hipXLength>0){
            hipX /= hipXLength;
        }
        else {
            NUClear::log<NUClear::DEBUG>("InverseKinematics::calculateLegJoints : targetLeg and ankleY parallel. This is unhandled at the moment. requested pose = \n", target);
            return positions;
        }
        arma::vec3 legPlaneTangent = arma::cross(ankleY, hipX); //Will be unit as ankleY and hipX are normal and unit

        ankleRoll = atan2(arma::dot(ankleX, legPlaneTangent),arma::dot(ankleX, hipX));

        arma::vec3 globalX = {1,0,0};
        arma::vec3 globalY = {0,1,0};
        arma::vec3 globalZ = {0,0,1};

        bool isAnkleAboveWaist = arma::dot(unitTargetLeg,globalZ)<0;

        float cosZandHipX = arma::dot(globalZ, hipX);
        bool hipRollPositive = cosZandHipX <= 0;
        arma::vec3 legPlaneGlobalZ = (isAnkleAboveWaist ? -1 : 1 ) * (globalZ - ( cosZandHipX * hipX));
        float legPlaneGlobalZLength = arma::norm(legPlaneGlobalZ, 2);
        if (legPlaneGlobalZLength>0){
           legPlaneGlobalZ /= legPlaneGlobalZLength;
        }

        float cosHipRoll = arma::dot(legPlaneGlobalZ, globalZ);
        // TODO: check if cosHipRoll is between 1 and -1
        hipRoll = (hipRollPositive ? 1 : -1) * acos(cosHipRoll);


        float phi4 = M_PI - knee - lowerLeg;
        //Superposition values:
        float sinPIminusPhi2 = std::sin(M_PI - phi2);
        arma::vec3 unitUpperLeg = unitTargetLeg * (std::sin(phi2 - phi4) / sinPIminusPhi2) + ankleY * (std::sin(phi4) / sinPIminusPhi2);
        bool isHipPitchPositive = dot(hipX,cross(unitUpperLeg, legPlaneGlobalZ))>=0;

        hipPitch = (isHipPitchPositive ? 1 : -1) * acos(arma::dot(legPlaneGlobalZ, unitUpperLeg));

        arma::vec3 hipXProjected = (isAnkleAboveWaist ? -1 : 1) * hipX;  //If leg is above waist then hipX is pointing in the wrong direction in the xy plane
        hipXProjected[2] = 0;
        hipXProjected /= arma::norm(hipXProjected, 2);
        bool isHipYawPositive = arma::dot(hipXProjected,globalY)>=0;

        hipYaw = (isHipYawPositive ? 1 : -1) * acos(arma::dot( hipXProjected,globalX));

        if (limb == messages::input::LimbID::LEFT_LEG) {
            positions.push_back(std::make_pair(messages::input::ServoID::L_HIP_YAW, -hipYaw));
            positions.push_back(std::make_pair(messages::input::ServoID::L_HIP_ROLL, hipRoll));
            positions.push_back(std::make_pair(messages::input::ServoID::L_HIP_PITCH, -hipPitch));
            positions.push_back(std::make_pair(messages::input::ServoID::L_KNEE, M_PI - knee));
            positions.push_back(std::make_pair(messages::input::ServoID::L_ANKLE_PITCH, -anklePitch));
            positions.push_back(std::make_pair(messages::input::ServoID::L_ANKLE_ROLL, ankleRoll));
        }
        else {
            positions.push_back(std::make_pair(messages::input::ServoID::R_HIP_YAW, (RobotKinematicModel::Leg::LEFT_TO_RIGHT_HIP_YAW) * -hipYaw));
            positions.push_back(std::make_pair(messages::input::ServoID::R_HIP_ROLL, (RobotKinematicModel::Leg::LEFT_TO_RIGHT_HIP_ROLL) * hipRoll));
            positions.push_back(std::make_pair(messages::input::ServoID::R_HIP_PITCH, (RobotKinematicModel::Leg::LEFT_TO_RIGHT_HIP_PITCH) * -hipPitch));
            positions.push_back(std::make_pair(messages::input::ServoID::R_KNEE, (RobotKinematicModel::Leg::LEFT_TO_RIGHT_KNEE) * (M_PI - knee) ));
            positions.push_back(std::make_pair(messages::input::ServoID::R_ANKLE_PITCH, (RobotKinematicModel::Leg::LEFT_TO_RIGHT_ANKLE_PITCH) * -anklePitch));
            positions.push_back(std::make_pair(messages::input::ServoID::R_ANKLE_ROLL, (RobotKinematicModel::Leg::LEFT_TO_RIGHT_ANKLE_ROLL) * ankleRoll));
        }

        return positions;
    }

    template <typename RobotKinematicModel>
    std::vector<std::pair<messages::input::ServoID, float>> calculateLegJointsTeamDarwin(utility::math::matrix::Transform3D target, messages::input::LimbID limb) {
        target(2,3) += RobotKinematicModel::TEAMDARWINCHEST_TO_ORIGIN; // translate without regard to rotation
        // target = target.translateZ(RobotKinematicModel::Leg::FOOT_HEIGHT); THIS HAS BEEN WRONG THE WHOLE TIME!!!! THIS ASSUMES THE FOOT IS FLAT RELATIVE TO THE TORSO (WHICH IT ISN'T BECAUSE THE BODY IS TILTED)
        return calculateLegJoints<RobotKinematicModel>(target, limb);
    }

    template <typename RobotKinematicModel>
    std::vector<std::pair<messages::input::ServoID, float>> calculateLegJointsTeamDarwin(utility::math::matrix::Transform3D leftTarget, utility::math::matrix::Transform3D rightTarget) {
        auto joints = calculateLegJointsTeamDarwin<RobotKinematicModel>(leftTarget, messages::input::LimbID::LEFT_LEG);
        auto joints2 = calculateLegJointsTeamDarwin<RobotKinematicModel>(rightTarget, messages::input::LimbID::RIGHT_LEG);
        joints.insert(joints.end(), joints2.begin(), joints2.end());
        return joints;
    }

    template <typename RobotKinematicModel>
    std::vector< std::pair<messages::input::ServoID, float> > calculateHeadJoints(arma::vec3 cameraUnitVector){
        std::vector< std::pair<messages::input::ServoID, float> > positions;
        positions.push_back(std::make_pair(messages::input::ServoID::HEAD_YAW, atan2(cameraUnitVector[1],cameraUnitVector[0]) ));
        positions.push_back(std::make_pair(messages::input::ServoID::HEAD_PITCH, atan2(-cameraUnitVector[2], std::sqrt(cameraUnitVector[0]*cameraUnitVector[0]+cameraUnitVector[1]*cameraUnitVector[1])) ));
        return positions;
    }

    template <typename RobotKinematicModel>
    inline arma::vec2 calculateHeadJointsToLookAt(arma::vec3 groundPoint, const utility::math::matrix::Transform3D camToGround, const utility::math::matrix::Transform3D orientationBodyToGround){
    // TODO: Find point that is invariant under head position.
        arma::vec3 cameraPosition = camToGround.submat(0,3,2,3);
        arma::vec3 groundSpaceLookVector = groundPoint - cameraPosition;
        arma::vec3 lookVector = orientationBodyToGround.submat(0,0,2,2).t() * groundSpaceLookVector;
        arma::vec3 lookVectorSpherical = utility::math::coordinates::cartesianToSpherical(lookVector);

        return lookVectorSpherical.rows(1,2);
    }

} // kinematics
}  // motion
}  // utility

#endif  // UTILITY_MOTION_INVERSEKINEMATICS_H
