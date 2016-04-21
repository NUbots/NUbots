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
#include <cmath>
#include <chrono>
#include <nuclear>
#include <limits>

#include "utility/math/matrix/Transform3D.h"
#include "utility/math/coordinates.h"
#include "utility/motion/RobotModels.h"
#include "utility/motion/ForwardKinematics.h"
#include "message/input/ServoID.h"
#include "message/input/Sensors.h"
#include "utility/math/angle.h"
#include "message/behaviour/Action.h"

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
    bool legPoseValid(utility::math::matrix::Transform3D target, message::input::LimbID limb) {
        const float HIP_OFFSET_Y = RobotKinematicModel::Leg::HIP_OFFSET_Y;
        const float HIP_OFFSET_Z = RobotKinematicModel::Leg::HIP_OFFSET_Z;
        const float HIP_OFFSET_X = RobotKinematicModel::Leg::HIP_OFFSET_X;
        const float UPPER_LEG_LENGTH = RobotKinematicModel::Leg::UPPER_LEG_LENGTH;
        const float LOWER_LEG_LENGTH = RobotKinematicModel::Leg::LOWER_LEG_LENGTH;

        //Translate up foot
        auto targetLeg = target.translate(arma::vec3({0,0,RobotKinematicModel::Leg::FOOT_HEIGHT}));

        //Remove hip offset
        int negativeIfRight = (limb == message::input::LimbID::RIGHT_LEG) ? -1 : 1;
        arma::vec3 hipOffset = { HIP_OFFSET_X, negativeIfRight * HIP_OFFSET_Y, -HIP_OFFSET_Z};
        targetLeg.translation() -= hipOffset;

        float length = arma::norm(targetLeg.translation());
        float maxLegLength = UPPER_LEG_LENGTH + LOWER_LEG_LENGTH;
        return (length < maxLegLength);
    }

    template <typename RobotKinematicModel>
    std::vector<std::pair<message::input::ServoID, float>> calculateLegJoints(utility::math::matrix::Transform3D target, message::input::LimbID limb) {
        const float LENGTH_BETWEEN_LEGS = RobotKinematicModel::Leg::LENGTH_BETWEEN_LEGS;
        const float DISTANCE_FROM_BODY_TO_HIP_JOINT = RobotKinematicModel::Leg::HIP_OFFSET_Z;
        const float HIP_OFFSET_X = RobotKinematicModel::Leg::HIP_OFFSET_X;
        const float UPPER_LEG_LENGTH = RobotKinematicModel::Leg::UPPER_LEG_LENGTH;
        const float LOWER_LEG_LENGTH = RobotKinematicModel::Leg::LOWER_LEG_LENGTH;

        std::vector<std::pair<message::input::ServoID, float> > positions;

        float hipYaw = 0;
        float hipRoll = 0;
        float hipPitch = 0;
        float knee = 0;
        float anklePitch = 0;
        float ankleRoll = 0;

        //Correct for input referencing the bottom of the foot
        target = target.translate(arma::vec3({0,0,RobotKinematicModel::Leg::FOOT_HEIGHT}));

        //TODO remove this. It was due to wrong convention use
        utility::math::matrix::Transform3D inputCoordinatesToCalcCoordinates;
        inputCoordinatesToCalcCoordinates << 0<< 1<< 0<< 0<< arma::endr
                                          << 1<< 0<< 0<< 0<< arma::endr
                                          << 0<< 0<<-1<< 0<< arma::endr
                                          << 0<< 0<< 0<< 1;
        //Rotate input position from standard robot coords to foot coords
        // NUClear::log<NUClear::DEBUG>("Target Original\n", target);
        arma::vec4 fourthColumn = inputCoordinatesToCalcCoordinates * target.col(3);
        target = inputCoordinatesToCalcCoordinates * target * inputCoordinatesToCalcCoordinates.t();
        target.col(3) = fourthColumn;
        // NUClear::log<NUClear::DEBUG>("Target Final\n", target);

        //swap legs if needed
        if (limb != message::input::LimbID::LEFT_LEG) {
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

        float length = arma::norm(targetLeg);
        float maxLegLength = UPPER_LEG_LENGTH + LOWER_LEG_LENGTH;
        if (length > maxLegLength){
            // NUClear::log<NUClear::WARN>("InverseKinematics::calculateLegJoints : !!! WARNING !!! Requested position beyond leg reach.\n Scaling back requested vector from length ",length, " to ", maxLegLength);
            targetLeg = targetLeg * (maxLegLength)/length;
            length = arma::norm(targetLeg);
        }
        // NUClear::log<NUClear::DEBUG>("Length: ", length);
        float sqrLength = length * length;
        float sqrUpperLeg = UPPER_LEG_LENGTH * UPPER_LEG_LENGTH;
        float sqrLowerLeg = LOWER_LEG_LENGTH * LOWER_LEG_LENGTH;

        float cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH);
       // NUClear::log<NUClear::DEBUG>("Cos Knee: ", cosKnee);
        // TODO: check if cosKnee is between 1 and -1
        knee = std::acos(std::fmax(std::fmin(cosKnee,1),-1));
       // NUClear::log<NUClear::DEBUG>("Knee: ", knee);

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

        if (limb == message::input::LimbID::LEFT_LEG) {
            positions.push_back(std::make_pair(message::input::ServoID::L_HIP_YAW, -hipYaw));
            positions.push_back(std::make_pair(message::input::ServoID::L_HIP_ROLL, hipRoll));
            positions.push_back(std::make_pair(message::input::ServoID::L_HIP_PITCH, -hipPitch));
            positions.push_back(std::make_pair(message::input::ServoID::L_KNEE, M_PI - knee));
            positions.push_back(std::make_pair(message::input::ServoID::L_ANKLE_PITCH, -anklePitch));
            positions.push_back(std::make_pair(message::input::ServoID::L_ANKLE_ROLL, ankleRoll));
        }
        else {
            positions.push_back(std::make_pair(message::input::ServoID::R_HIP_YAW, (RobotKinematicModel::Leg::LEFT_TO_RIGHT_HIP_YAW) * -hipYaw));
            positions.push_back(std::make_pair(message::input::ServoID::R_HIP_ROLL, (RobotKinematicModel::Leg::LEFT_TO_RIGHT_HIP_ROLL) * hipRoll));
            positions.push_back(std::make_pair(message::input::ServoID::R_HIP_PITCH, (RobotKinematicModel::Leg::LEFT_TO_RIGHT_HIP_PITCH) * -hipPitch));
            positions.push_back(std::make_pair(message::input::ServoID::R_KNEE, (RobotKinematicModel::Leg::LEFT_TO_RIGHT_KNEE) * (M_PI - knee) ));
            positions.push_back(std::make_pair(message::input::ServoID::R_ANKLE_PITCH, (RobotKinematicModel::Leg::LEFT_TO_RIGHT_ANKLE_PITCH) * -anklePitch));
            positions.push_back(std::make_pair(message::input::ServoID::R_ANKLE_ROLL, (RobotKinematicModel::Leg::LEFT_TO_RIGHT_ANKLE_ROLL) * ankleRoll));
        }

        return positions;
    }

    template <typename RobotKinematicModel>
    std::vector<std::pair<message::input::ServoID, float>> calculateLegJoints(utility::math::matrix::Transform3D leftTarget, utility::math::matrix::Transform3D rightTarget) {
        auto joints = calculateLegJoints<RobotKinematicModel>(leftTarget, message::input::LimbID::LEFT_LEG);
        auto joints2 = calculateLegJoints<RobotKinematicModel>(rightTarget, message::input::LimbID::RIGHT_LEG);
        joints.insert(joints.end(), joints2.begin(), joints2.end());
        return joints;
    }

    template <typename RobotKinematicModel>
    std::vector<std::pair<message::input::ServoID, float>> calculateLegJointsTeamDarwin(utility::math::matrix::Transform3D target, message::input::LimbID limb) {
        target(2,3) += RobotKinematicModel::TEAMDARWINCHEST_TO_ORIGIN; // translate without regard to rotation
        // target = target.translateZ(RobotKinematicModel::Leg::FOOT_HEIGHT); THIS HAS BEEN WRONG THE WHOLE TIME!!!! THIS ASSUMES THE FOOT IS FLAT RELATIVE TO THE TORSO (WHICH IT ISN'T BECAUSE THE BODY IS TILTED)
        return calculateLegJoints<RobotKinematicModel>(target, limb);
    }

    template <typename RobotKinematicModel>
    std::vector<std::pair<message::input::ServoID, float>> calculateLegJointsTeamDarwin(utility::math::matrix::Transform3D leftTarget, utility::math::matrix::Transform3D rightTarget) {
        auto joints = calculateLegJointsTeamDarwin<RobotKinematicModel>(leftTarget, message::input::LimbID::LEFT_LEG);
        auto joints2 = calculateLegJointsTeamDarwin<RobotKinematicModel>(rightTarget, message::input::LimbID::RIGHT_LEG);
        joints.insert(joints.end(), joints2.begin(), joints2.end());
        return joints;
    }

    template <typename RobotKinematicModel>
    std::vector< std::pair<message::input::ServoID, float> > calculateHeadJoints(arma::vec3 cameraUnitVector){
        std::vector< std::pair<message::input::ServoID, float> > positions;
        positions.push_back(std::make_pair(message::input::ServoID::HEAD_YAW, atan2(cameraUnitVector[1],cameraUnitVector[0]) ));
        positions.push_back(std::make_pair(message::input::ServoID::HEAD_PITCH, atan2(-cameraUnitVector[2], std::sqrt(cameraUnitVector[0]*cameraUnitVector[0]+cameraUnitVector[1]*cameraUnitVector[1])) ));
        return positions;
    }

    template <typename RobotKinematicModel>
    inline arma::vec2 calculateHeadJointsToLookAt(arma::vec3 groundPoint, const utility::math::matrix::Transform3D& camToGround, const utility::math::matrix::Transform3D& orientationBodyToGround){
    // TODO: Find point that is invariant under head position.
        arma::vec3 cameraPosition = camToGround.submat(0,3,2,3);
        arma::vec3 groundSpaceLookVector = groundPoint - cameraPosition;
        arma::vec3 lookVector = orientationBodyToGround.submat(0,0,2,2).t() * groundSpaceLookVector;
        arma::vec3 lookVectorSpherical = utility::math::coordinates::cartesianToSpherical(lookVector);

        return lookVectorSpherical.rows(1,2);
    }

    template <typename RobotKinematicModel>
    inline arma::vec2 headAnglesToSeeGroundPoint(const arma::vec2& gpos, const message::input::Sensors& sensors){
        arma::vec3 groundPos_ground = {gpos[0],gpos[1],0};
        return calculateHeadJointsToLookAt<RobotKinematicModel>(groundPos_ground, sensors.orientationCamToGround, sensors.orientationBodyToGround);
    }

    template <typename RobotKinematicModel>
    std::vector<std::pair<message::input::ServoID, float>> setHeadPoseFromFeet(const utility::math::matrix::Transform3D& cameraToFeet, const float& footSeparation, const float& bodyAngle){
        //Get camera pose relative to body
        // arma::vec3 euler = cameraToFeet.rotation().eulerAngles();
        // float headPitch = euler[1] - bodyAngle;
        // float headYaw = euler[2];
        arma::vec3 gaze = cameraToFeet.rotation().col(0);
        auto headJoints = utility::motion::kinematics::calculateHeadJoints<RobotKinematicModel>(gaze);
        float headPitch = std::numeric_limits<float>::quiet_NaN();
        float headYaw = std::numeric_limits<float>::quiet_NaN();
        for(auto joint : headJoints){
            switch(joint.first){
                case message::input::ServoID::HEAD_PITCH:
                    headPitch = joint.second;
                    break;
                case message::input::ServoID::HEAD_YAW:
                    headYaw = joint.second;
                    break;
                default:
                    NUClear::log<NUClear::ERROR>(__FILE__,__LINE__,"Joints for head returned unexpected values");
                    throw std::exception();
            }
        }
        if( std::isnan(headPitch * headPitch) ){
            NUClear::log<NUClear::ERROR>(__FILE__,__LINE__,"Joints for head missing values!!!");
            throw std::exception();
        }

        auto headPoses = utility::motion::kinematics::calculateHeadJointPosition<RobotKinematicModel>(headPitch,
                                                                                                     headYaw,
                                                                                                     message::input::ServoID::HEAD_PITCH);
        auto cameraToBody = headPoses[message::input::ServoID::HEAD_PITCH];

        //Compute foot poses
        utility::math::matrix::Transform3D F_c = cameraToBody * cameraToFeet.i();
        utility::math::matrix::Transform3D F_l = F_c.translateY(footSeparation / 2.0);
        utility::math::matrix::Transform3D F_r = F_c.translateY(-footSeparation / 2.0);

        //Get associated joint angles
        auto joints = calculateLegJoints<RobotKinematicModel>(F_l, message::input::LimbID::LEFT_LEG);
        auto joints2 = calculateLegJoints<RobotKinematicModel>(F_r, message::input::LimbID::RIGHT_LEG);
        joints.insert(joints.end(), joints2.begin(), joints2.end());
        joints.insert(joints.end(),headJoints.begin(),headJoints.end());

        // return joints;
        return headJoints;
    }

    template <typename RobotKinematicModel>
    std::vector<std::pair<message::input::ServoID, float>> setArm(const arma::vec3& pos, bool left, int number_of_iterations = 300, arma::vec3 angleHint = arma::zeros(3)){
        message::input::ServoID SHOULDER_PITCH, SHOULDER_ROLL, ELBOW;
        int negativeIfRight = 1;

        if(static_cast<bool>(left)){
            SHOULDER_PITCH = message::input::ServoID::L_SHOULDER_PITCH;
            SHOULDER_ROLL = message::input::ServoID::L_SHOULDER_ROLL;
            ELBOW = message::input::ServoID::L_ELBOW;
        } else {
            SHOULDER_PITCH = message::input::ServoID::R_SHOULDER_PITCH;
            SHOULDER_ROLL = message::input::ServoID::R_SHOULDER_ROLL;
            ELBOW = message::input::ServoID::R_ELBOW;
            negativeIfRight = -1;
        }

        auto start_compute = NUClear::clock::now();

        //Initial guess for angles
        arma::vec3 angles = angleHint;
        arma::vec3 X = {0,0,0};
        int i = 0;
        for(; i < number_of_iterations; i++){
            X = calculateArmPosition<RobotKinematicModel>(angles, left);
            arma::vec3 dX = pos - X;
            
            arma::mat33 J = calculateArmJacobian<RobotKinematicModel>(angles, left);
            // std::cout << "pos = " << pos.t() << std::endl;
            // std::cout << "X = " << X.t() << std::endl;
            // std::cout << "dX = " << dX.t() << std::endl;
            // std::cout << "angles = " << angles.t() << std::endl;
            // std::cout << "error = " << arma::norm(dX) << std::endl;
            if(arma::norm(dX) < 0.001){
                break;
            }
            arma::vec3 dAngles = J.t() * dX;// * std::max((100 - i),1);
            angles = dAngles + angles;
        }
        auto end_compute = NUClear::clock::now();
        std::cout << "Computation Time (ms) = " << std::chrono::duration_cast<std::chrono::microseconds>(end_compute - start_compute).count() * 1e-3 << std::endl;
        // std::cout << "Final angles = " << angles.t() << std::endl;
        // std::cout << "Final position = " << X.t() << std::endl;
        // std::cout << "Goal position = " << pos.t() << std::endl;
        std::cout << "Final error = " << arma::norm(pos-X) << std::endl;
        // std::cout << "Iterations = " << i << std::endl;

        
        std::vector<std::pair<message::input::ServoID, float> > joints;
        joints.push_back(std::make_pair(SHOULDER_PITCH,utility::math::angle::normalizeAngle(angles[0])));
        joints.push_back(std::make_pair(SHOULDER_ROLL,utility::math::angle::normalizeAngle(angles[1])));
        joints.push_back(std::make_pair(ELBOW,utility::math::angle::normalizeAngle(angles[2])));
        return joints;
    } 

    template <typename RobotKinematicModel>
    std::vector<std::pair<message::input::ServoID, float>> setArmApprox(const arma::vec3& pos, bool left){
        //Setup variables
        message::input::ServoID SHOULDER_PITCH, SHOULDER_ROLL, ELBOW;
        int negativeIfRight = 1;
        if(left){
            SHOULDER_PITCH = message::input::ServoID::L_SHOULDER_PITCH;
            SHOULDER_ROLL = message::input::ServoID::L_SHOULDER_ROLL;
            ELBOW = message::input::ServoID::L_ELBOW;
        } else {
            SHOULDER_PITCH = message::input::ServoID::R_SHOULDER_PITCH;
            SHOULDER_ROLL = message::input::ServoID::R_SHOULDER_ROLL;
            ELBOW = message::input::ServoID::R_ELBOW;
            negativeIfRight = -1;
        }
        //Compute Angles
        float pitch,roll,elbow = 0;

        arma::vec3 shoulderPos = {  
            RobotKinematicModel::Arm::SHOULDER_X_OFFSET,
            negativeIfRight * RobotKinematicModel::Arm::DISTANCE_BETWEEN_SHOULDERS / 2,
            RobotKinematicModel::Arm::SHOULDER_Z_OFFSET
        };

        arma::vec3 handFromShoulder = pos - shoulderPos;
        // std::cout << (left ? "left" : "right" ) << " shoulderPos = " << shoulderPos.t();
        // std::cout << (left ? "right" : "left" ) << " pos = " << pos.t();
        // std::cout << (left ? "left" : "right" ) << " handFromShoulder = " << handFromShoulder.t();

        //ELBOW
        float extensionLength = arma::norm(handFromShoulder);
        float upperArmLength = RobotKinematicModel::Arm::UPPER_ARM_LENGTH;
        float lowerArmLength = RobotKinematicModel::Arm::LOWER_ARM_LENGTH;
        float sqrUpperArmLength = upperArmLength * upperArmLength;
        float sqrLowerArmLength = lowerArmLength * lowerArmLength;
        float sqrExtensionLength = extensionLength * extensionLength;
        float cosElbow = (sqrUpperArmLength + sqrLowerArmLength - sqrExtensionLength) / (2 * upperArmLength * lowerArmLength);
        elbow = -M_PI + std::acos(std::fmax(std::fmin(cosElbow,1),-1));

        //SHOULDER PITCH
        float cosPitAngle = (sqrUpperArmLength + sqrExtensionLength - sqrLowerArmLength) / (2 * upperArmLength * extensionLength);
        pitch = std::acos(std::fmax(std::fmin(cosPitAngle,1),-1)) + std::atan2(-handFromShoulder[2],handFromShoulder[0]);
        //SHOULDER ROLL
        arma::vec3 pitchlessHandFromShoulder = utility::math::matrix::Rotation3D::createRotationY(-pitch) * handFromShoulder;
        roll = std::atan2(pitchlessHandFromShoulder[1],pitchlessHandFromShoulder[0]);

        //Write to servo list
        std::vector<std::pair<message::input::ServoID, float> > joints;
        joints.push_back(std::make_pair(SHOULDER_PITCH,utility::math::angle::normalizeAngle(pitch)));
        joints.push_back(std::make_pair(SHOULDER_ROLL,utility::math::angle::normalizeAngle(roll)));
        joints.push_back(std::make_pair(ELBOW,utility::math::angle::normalizeAngle(elbow)));
        // std::cout << (left ? "left" : "right" ) << " roll,pitch,elbow (deg) = " << 180 * roll / M_PI << ", " << 180 * pitch / M_PI << ", " << 180 * elbow / M_PI << std::endl;
        return joints;
    }


} // kinematics
}  // motion
}  // utility

#endif  // UTILITY_MOTION_INVERSEKINEMATICS_H
