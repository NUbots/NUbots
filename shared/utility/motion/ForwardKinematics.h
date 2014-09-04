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

#ifndef UTILITY_MOTION_FORWARDKINEMATICS_H
#define UTILITY_MOTION_FORWARDKINEMATICS_H

#include <vector>
#include <armadillo>
#include <nuclear_bits/LogLevel.h>
#include <cmath>
#include <nuclear>

#include "InverseKinematics.h"

#include "utility/math/matrix.h"
#include "utility/motion/RobotModels.h"

#include "messages/input/Sensors.h"
#include "messages/input/ServoID.h"

namespace utility {
namespace motion {
namespace kinematics {


    template <typename RobotKinematicModel>
    inline std::map<messages::input::ServoID, arma::mat44> calculateHeadJointPosition(const messages::input::Sensors& sensors, messages::input::ServoID servoID){
        std::map<messages::input::ServoID, arma::mat44> positions;

        arma::mat44 runningTransform = arma::eye(4, 4);
        float HEAD_PITCH = sensors.servos[static_cast<int>(messages::input::ServoID::HEAD_PITCH)].presentPosition;
        float HEAD_YAW =  sensors.servos[static_cast<int>(messages::input::ServoID::HEAD_YAW)].presentPosition;
        arma::vec3 NECK_POS = {RobotKinematicModel::Head::NECK_BASE_POS_FROM_ORIGIN[0],
                               RobotKinematicModel::Head::NECK_BASE_POS_FROM_ORIGIN[1],
                               RobotKinematicModel::Head::NECK_BASE_POS_FROM_ORIGIN[2]};
        float NECK_LENGTH = RobotKinematicModel::Head::NECK_LENGTH;
        arma::vec3 NECK_TO_CAMERA = {RobotKinematicModel::Head::NECK_TO_CAMERA[0],
                               RobotKinematicModel::Head::NECK_TO_CAMERA[1],
                               RobotKinematicModel::Head::NECK_TO_CAMERA[2]};

        //Translate to base of neck from origin
        runningTransform *= utility::math::matrix::translationMatrix(NECK_POS);
        //Rotate to face out of base of neck
        runningTransform *= utility::math::matrix::yRotationMatrix(-M_PI_2, 4);
        //Rotate head in yaw axis
        runningTransform *= utility::math::matrix::xRotationMatrix(HEAD_YAW, 4);
        //Translate to top of neck (i.e. next motor axle)
        runningTransform *= utility::math::matrix::translationMatrix(arma::vec3({NECK_LENGTH, 0, 0}));
        //YAW
        //Return the basis pointing out of the top of the torso with z pointing out the back of the neck. Pos is top of neck (at hip pitch motor)
        positions[messages::input::ServoID::HEAD_YAW] = runningTransform;
        if(servoID == messages::input::ServoID::HEAD_YAW) {
            return positions;
        }

        //Rotate to face forward direction of neck
        runningTransform *= utility::math::matrix::yRotationMatrix(M_PI_2, 4);
        //Rotate pitch
        runningTransform *= utility::math::matrix::yRotationMatrix(HEAD_PITCH, 4);
        //Translate to camera
        runningTransform *= utility::math::matrix::translationMatrix(NECK_TO_CAMERA);
        //Rotate to set x to camera vector
        runningTransform *= utility::math::matrix::yRotationMatrix(RobotKinematicModel::Head::CAMERA_DECLINATION_ANGLE_OFFSET, 4);
        //PITCH
        //Return basis pointing along camera vector (ie x is camera vector, z out of top of head). Pos at camera position
        positions[messages::input::ServoID::HEAD_PITCH] = runningTransform;
        return positions;
    }

    /*! @brief
        @NOTE read " runningTransform *= utility::math::matrix::_RotationMatrix(angle, 4); " as "Rotate the running transform about its local _ coordinate by angle."
        @return Returns basis matrix for position of end of limb controlled by the specified motor.

        The basis 'faces' down its x axis.
    */
    template <typename RobotKinematicModel>
    inline std::map<messages::input::ServoID, arma::mat44> calculateLegJointPosition(const messages::input::Sensors& sensors, messages::input::ServoID servoID, Side isLeft){
        std::map<messages::input::ServoID, arma::mat44> positions;
        arma::mat44 runningTransform = arma::eye(4, 4);
        //Variables to mask left and right leg differences:
        messages::input::ServoID HIP_YAW, HIP_ROLL, HIP_PITCH, KNEE, ANKLE_PITCH, ANKLE_ROLL;
        int negativeIfRight = 1;

        if(static_cast<bool>(isLeft)){
            HIP_YAW = messages::input::ServoID::L_HIP_YAW;
            HIP_ROLL = messages::input::ServoID::L_HIP_ROLL;
            HIP_PITCH = messages::input::ServoID::L_HIP_PITCH;
            KNEE = messages::input::ServoID::L_KNEE;
            ANKLE_PITCH = messages::input::ServoID::L_ANKLE_PITCH;
            ANKLE_ROLL = messages::input::ServoID::L_ANKLE_ROLL;
        } else {
            HIP_YAW = messages::input::ServoID::R_HIP_YAW;
            HIP_ROLL = messages::input::ServoID::R_HIP_ROLL;
            HIP_PITCH = messages::input::ServoID::R_HIP_PITCH;
            KNEE = messages::input::ServoID::R_KNEE;
            ANKLE_PITCH = messages::input::ServoID::R_ANKLE_PITCH;
            ANKLE_ROLL = messages::input::ServoID::R_ANKLE_ROLL;
            negativeIfRight = -1;
        }

        arma::mat44 hipPos = arma::eye(4, 4);
        hipPos.col(3)= arma::vec({ RobotKinematicModel::Leg::HIP_OFFSET_X, negativeIfRight * RobotKinematicModel::Leg::LENGTH_BETWEEN_LEGS/2, -RobotKinematicModel::Leg::HIP_OFFSET_Z,1});
        runningTransform *= hipPos;
        //Rotate to face down the leg (see above for definitions of terms, including 'facing')
        runningTransform *= utility::math::matrix::yRotationMatrix(M_PI_2, 4);
        //Using right hand rule along global z gives positive direction of yaw:
        runningTransform *= utility::math::matrix::xRotationMatrix(-sensors.servos[static_cast<int>(HIP_YAW)].presentPosition, 4);
        //Return basis facing from body to hip centre (down) with z aligned with the axis of the hip roll motor axis. Position at hip joint
        positions[HIP_YAW] = runningTransform;
        if(servoID == HIP_YAW) {
            return positions;
        }

        runningTransform *= utility::math::matrix::zRotationMatrix(sensors.servos[static_cast<int>(HIP_ROLL)].presentPosition, 4);
        //Return basis facing down leg plane, with z oriented through axis of roll motor. Position still hip joint
        positions[HIP_ROLL] = runningTransform;
        if(servoID == HIP_ROLL) {
            return positions;
        }

        //Rotate to face down upper leg
        runningTransform *= utility::math::matrix::yRotationMatrix(sensors.servos[static_cast<int>(HIP_PITCH)].presentPosition , 4);
        //Translate down upper leg
        runningTransform *= utility::math::matrix::translationMatrix(arma::vec3({RobotKinematicModel::Leg::UPPER_LEG_LENGTH, 0, 0}));
        //Return basis faces down upper leg, with z out of front of thigh. Pos = knee axis centre
        positions[HIP_PITCH] = runningTransform;
        if(servoID == HIP_PITCH) {
            return positions;
        }


        //Rotate to face down lower leg
        runningTransform *= utility::math::matrix::yRotationMatrix(sensors.servos[static_cast<int>(KNEE)].presentPosition , 4);
        //Translate down lower leg
        runningTransform *= utility::math::matrix::translationMatrix(arma::vec3({RobotKinematicModel::Leg::UPPER_LEG_LENGTH, 0, 0}));
        //Return basis facing down lower leg, with z out of front of shin. Pos = ankle axis centre
        positions[KNEE] = runningTransform;
        if(servoID == KNEE) {
            return positions;
        }


        //Rotate to face down foot (pitch)
        runningTransform *= utility::math::matrix::yRotationMatrix(sensors.servos[static_cast<int>(ANKLE_PITCH)].presentPosition , 4);
        //Return basis facing pitch down to foot with z out the front of the foot. Pos = ankle axis centre
        positions[ANKLE_PITCH] = runningTransform;
        if(servoID == ANKLE_PITCH) {
            return positions;
        }

        //Rotate to face down foot (roll)
        runningTransform *= utility::math::matrix::zRotationMatrix(sensors.servos[static_cast<int>(ANKLE_ROLL)].presentPosition , 4);
        //Rotate so x faces towar toes
        runningTransform *= utility::math::matrix::yRotationMatrix(-M_PI_2, 4);
        //Translate to ground
        runningTransform *= utility::math::matrix::translationMatrix(arma::vec3({0, 0, -RobotKinematicModel::Leg::FOOT_HEIGHT}));
        //Return basis with x out of the front of the toe and z out the top of foot. Pos = ankle axis centre projected to ground
        positions[ANKLE_ROLL] = runningTransform;
        return positions;
    }

    /*! @brief
        @NOTE read " runningTransform *= utility::math::matrix::_RotationMatrix(angle, 4); " as "Rotate the running transform about its local _ coordinate by angle."
        @return Returns basis matrix for position of end of limb controlled by the specified motor.

        The basis 'faces' down its x axis.
    */
    template <typename RobotKinematicModel>
    inline std::map<messages::input::ServoID, arma::mat44> calculateArmJointPosition(const messages::input::Sensors& sensors, messages::input::ServoID servoID, Side isLeft){
        std::map<messages::input::ServoID, arma::mat44> positions;
        arma::mat44 runningTransform = arma::eye(4, 4);
        //Variables to mask left and right leg differences:
        messages::input::ServoID SHOULDER_PITCH, SHOULDER_ROLL, ELBOW;
        int negativeIfRight = 1;

        if(static_cast<bool>(isLeft)){
            SHOULDER_PITCH = messages::input::ServoID::L_SHOULDER_PITCH;
            SHOULDER_ROLL = messages::input::ServoID::L_SHOULDER_ROLL;
            ELBOW = messages::input::ServoID::L_ELBOW;
        } else {
            SHOULDER_PITCH = messages::input::ServoID::R_SHOULDER_PITCH;
            SHOULDER_ROLL = messages::input::ServoID::R_SHOULDER_ROLL;
            ELBOW = messages::input::ServoID::R_ELBOW;
            negativeIfRight = -1;
        }

        //Translate to shoulder
        runningTransform *= utility::math::matrix::translationMatrix(arma::vec3({RobotKinematicModel::Arm::SHOULDER_X_OFFSET,
                                                                                 negativeIfRight * RobotKinematicModel::Arm::DISTANCE_BETWEEN_SHOULDERS / 2.0,
                                                                                 RobotKinematicModel::Arm::SHOULDER_Z_OFFSET}));
        //Rotate about shoulder pitch
        runningTransform *= utility::math::matrix::yRotationMatrix(sensors.servos[static_cast<int>(SHOULDER_PITCH)].presentPosition, 4);
        //Translate to end of shoulder part
        runningTransform *= utility::math::matrix::translationMatrix(arma::vec3({RobotKinematicModel::Arm::SHOULDER_LENGTH,
                                                                                 negativeIfRight * RobotKinematicModel::Arm::SHOULDER_WIDTH,
                                                                                 RobotKinematicModel::Arm::SHOULDER_HEIGHT}));
        //Return matrix pointing down shoulder, y same as global y. Pos = at centre of shoulder roll joint
        positions[SHOULDER_PITCH] = runningTransform;
        if(servoID == SHOULDER_PITCH){
            return positions;
        }

        //Rotate by the shoulder roll
        runningTransform *= utility::math::matrix::zRotationMatrix(sensors.servos[static_cast<int>(SHOULDER_ROLL)].presentPosition, 4);
        //Translate to centre of next joint
        runningTransform *= utility::math::matrix::translationMatrix(arma::vec3({RobotKinematicModel::Arm::UPPER_ARM_LENGTH,
                                                                                 negativeIfRight * RobotKinematicModel::Arm::UPPER_ARM_Y_OFFSET,
                                                                                 RobotKinematicModel::Arm::UPPER_ARM_Z_OFFSET}));
        positions[SHOULDER_ROLL] = runningTransform;
        if(servoID == SHOULDER_ROLL){
            return positions;
        }

        //Rotate by the elbow angle
        runningTransform *= utility::math::matrix::yRotationMatrix(sensors.servos[static_cast<int>(ELBOW)].presentPosition, 4);
        //Translate to centre of end of arm, in line with joint
        runningTransform *= utility::math::matrix::translationMatrix(arma::vec3({RobotKinematicModel::Arm::LOWER_ARM_LENGTH,
                                                                                 negativeIfRight * RobotKinematicModel::Arm::LOWER_ARM_Y_OFFSET,
                                                                                 RobotKinematicModel::Arm::LOWER_ARM_Z_OFFSET}));
        positions[ELBOW] = runningTransform;
        return positions;
    }

    /*! @brief
    */
    template <typename RobotKinematicModel>
    inline std::map<messages::input::ServoID, arma::mat44> calculatePosition(const messages::input::Sensors& sensors, messages::input::ServoID servoID) {
        switch(servoID) {
            case messages::input::ServoID::HEAD_YAW:
            case messages::input::ServoID::HEAD_PITCH:
                return calculateHeadJointPosition<RobotKinematicModel>(sensors, servoID);
            case messages::input::ServoID::R_SHOULDER_PITCH:
            case messages::input::ServoID::R_SHOULDER_ROLL:
            case messages::input::ServoID::R_ELBOW:
                return calculateArmJointPosition<RobotKinematicModel>(sensors, servoID, Side::RIGHT);
            case messages::input::ServoID::L_SHOULDER_PITCH:
            case messages::input::ServoID::L_SHOULDER_ROLL:
            case messages::input::ServoID::L_ELBOW:
                return calculateArmJointPosition<RobotKinematicModel>(sensors, servoID, Side::LEFT);
            case messages::input::ServoID::R_HIP_YAW:
            case messages::input::ServoID::R_HIP_ROLL:
            case messages::input::ServoID::R_HIP_PITCH:
            case messages::input::ServoID::R_KNEE:
            case messages::input::ServoID::R_ANKLE_PITCH:
            case messages::input::ServoID::R_ANKLE_ROLL:
                return calculateLegJointPosition<RobotKinematicModel>(sensors, servoID, Side::RIGHT);
            case messages::input::ServoID::L_HIP_YAW:
            case messages::input::ServoID::L_HIP_ROLL:
            case messages::input::ServoID::L_HIP_PITCH:
            case messages::input::ServoID::L_KNEE:
            case messages::input::ServoID::L_ANKLE_PITCH:
            case messages::input::ServoID::L_ANKLE_ROLL:
                return calculateLegJointPosition<RobotKinematicModel>(sensors, servoID, Side::LEFT);
        }
        return std::map<messages::input::ServoID, arma::mat44>();
    }


    template <typename RobotKinematicModel>
    inline std::map<messages::input::ServoID, arma::mat44> calculateAllPositions(const messages::input::Sensors& sensors) {
        std::map<messages::input::ServoID, arma::mat44> result = calculatePosition<RobotKinematicModel>(sensors, messages::input::ServoID::L_ANKLE_ROLL);
        std::map<messages::input::ServoID, arma::mat44> rightLegPositions = calculatePosition<RobotKinematicModel>(sensors, messages::input::ServoID::R_ANKLE_ROLL);
        std::map<messages::input::ServoID, arma::mat44> headPositions = calculatePosition<RobotKinematicModel>(sensors, messages::input::ServoID::HEAD_PITCH);
        std::map<messages::input::ServoID, arma::mat44> leftArm = calculatePosition<RobotKinematicModel>(sensors, messages::input::ServoID::L_ELBOW);
        std::map<messages::input::ServoID, arma::mat44> rightArm = calculatePosition<RobotKinematicModel>(sensors, messages::input::ServoID::R_ELBOW);
        result.insert(leftArm.begin(), leftArm.end());
        result.insert(rightArm.begin(), rightArm.end());
        result.insert(rightLegPositions.begin(), rightLegPositions.end());
        result.insert(headPositions.begin(), headPositions.end());
        return result;
    }
    /*! @brief Adds up the mass vectors stored in the robot model and normalises the resulting position
        @return [x_com, y_com, z_com, total_mass] relative to the torso basis
    */
    template <typename RobotKinematicModel>
    inline arma::vec4 calculateCentreOfMass(const std::map<messages::input::ServoID, arma::mat44>& jointPositions, bool includeTorso){
        arma::vec4 totalMassVector;

        for(auto& joint : jointPositions){
            arma::vec4 massVector;
            for(size_t i = 0; i < 4; i++){
                massVector[i] = RobotKinematicModel::MassModel::masses[static_cast<int>(joint.first)][i];
            }
            //NUClear::log<NUClear::DEBUG>("calculateCentreOfMass - reading mass ", messages::input::stringFromId(joint.first), massVector);
            double jointMass = massVector[3];

            arma::mat44 massScaler = arma::eye(4, 4);
            massScaler.submat(0,0,2,2) *= jointMass;

            totalMassVector +=  joint.second * massScaler * massVector; // = m * local centre of mass in global robot coords
        }

        if(includeTorso){
            arma::vec4 massVector;
             for(size_t i = 0; i < 4; i++){
                massVector[i] = RobotKinematicModel::MassModel::masses[20][i];
            }
            //NUClear::log<NUClear::DEBUG>("calculateCentreOfMass - reading mass Torso", massVector);
            double jointMass = massVector[3];
            arma::mat44 massScaler = arma::eye(4, 4);
            massScaler.submat(0,0,2,2) *= jointMass;
            totalMassVector +=  massScaler * massVector; // = m * local centre of mass in global robot coords
        }

        arma::mat44 normaliser = arma::eye(4, 4);
        if(totalMassVector[3] > 0){
            normaliser.submat(0,0,2,2) *= 1 / totalMassVector[3];
            arma::vec4 result = normaliser * totalMassVector;
            return result;
        } else {
            NUClear::log<NUClear::ERROR>("ForwardKinematics::calculateCentreOfMass - Empty centre of mass request or no mass in mass model.");
            return arma::vec4();
        }
    }

    inline utility::math::geometry::Line calculateHorizon(const arma::mat33 groundToCamRotation, double cameraDistancePixels){
        arma::vec3 zGround = {0,0,1};
        arma::vec3 normal = groundToCamRotation * zGround;

        arma::vec3 xHead = {1,0,0};
        arma::vec3 yHead = {0,1,0};
        double phiX = std::acos(arma::dot(normal, xHead)) - M_PI_2;
        double phiY = std::acos(arma::dot(normal, yHead)) - M_PI_2;

        // TODO ask jake to fix this :P

        // Since I don't know how to math this properly, make two random points and make a line from those
        double m = std::tan(phiY);
        double b = cameraDistancePixels * -std::tan(phiX);

        return utility::math::geometry::Line(arma::vec2({ 0, b }), arma::vec2({ 1, m + b }));
    }

    inline arma::mat44 calculateBodyToGround(arma::vec3 groundNormal_body, double bodyHeight){
        arma::vec3 X = arma::vec{1,0,0};
        double projectXOnNormal = groundNormal_body[0];

        arma::vec3 groundMatrixX;
        arma::vec3 groundMatrixY;

        if(std::fabs(projectXOnNormal) == 1){
            //Then x is parallel to the ground normal and we need to use projection onto +/-z instead
            //If x parallel to normal, then use -z, if x antiparallel use z
            arma::vec3 Z =  arma::vec3{0, 0, (projectXOnNormal > 0 ? -1.0 : 1.0 )};
            double projectZOnNormal = arma::dot(Z, groundNormal_body);
            groundMatrixX = arma::normalise(Z - projectZOnNormal * groundNormal_body);
            groundMatrixY = arma::cross(groundNormal_body, groundMatrixX);
        } else {
            groundMatrixX = arma::normalise(X - projectXOnNormal * groundNormal_body);
            groundMatrixY = arma::cross(groundNormal_body, groundMatrixX);
        }

        arma::mat44 groundToBody = arma::eye(4, 4);
        groundToBody.submat(0,0,2,0) = groundMatrixX;
        groundToBody.submat(0,1,2,1) = groundMatrixY;
        groundToBody.submat(0,2,2,2) = groundNormal_body;
        groundToBody.submat(0,3,2,3) = arma::vec{0, 0, -bodyHeight};
        return utility::math::matrix::orthonormal44Inverse(groundToBody);
    }

    inline arma::mat22 calculateRobotToIMU(arma::mat33 orientation) {
        arma::vec3 xRobotImu = arma::mat33(orientation.t()).col(0);
        arma::vec2 projXRobot = arma::normalise(xRobotImu.rows(0,1));
        arma::vec2 projYRobot = arma::vec2({-projXRobot(1), projXRobot(0)});
        
        arma::mat22 robotToImu;
        robotToImu.col(0) = projXRobot;
        robotToImu.col(1) = projYRobot;

        return robotToImu;
    }

} // kinematics
}  // motion
}  // utility

#endif  // UTILITY_MOTION_FORWARDKINEMATICS_H