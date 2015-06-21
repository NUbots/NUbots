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
#include <cmath>
#include <nuclear>

#include "InverseKinematics.h"

#include "utility/math/matrix/Transform3D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/motion/RobotModels.h"

#include "messages/input/Sensors.h"
#include "messages/input/ServoID.h"

namespace utility {
namespace motion {
namespace kinematics {


    template <typename RobotKinematicModel>
    inline std::map<messages::input::ServoID, utility::math::matrix::Transform3D> calculateHeadJointPosition(const messages::input::Sensors& sensors, messages::input::ServoID servoID){
        std::map<messages::input::ServoID, utility::math::matrix::Transform3D> positions;

        utility::math::matrix::Transform3D runningTransform;
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
        runningTransform = runningTransform.translate(NECK_POS);
        //Rotate to face out of base of neck
        runningTransform = runningTransform.rotateY(-M_PI_2);
        //Rotate head in yaw axis
        runningTransform = runningTransform.rotateX(HEAD_YAW);
        //Translate to top of neck (i.e. next motor axle)
        runningTransform = runningTransform.translateX(NECK_LENGTH);
        //YAW
        //Return the basis pointing out of the top of the torso with z pointing out the back of the neck. Pos is top of neck (at hip pitch motor)
        positions[messages::input::ServoID::HEAD_YAW] = runningTransform;
        if(servoID == messages::input::ServoID::HEAD_YAW) {
            return positions;
        }

        //Rotate to face forward direction of neck
        runningTransform = runningTransform.rotateY(M_PI_2);
        //Rotate pitch
        runningTransform = runningTransform.rotateY(HEAD_PITCH);
        //Translate to camera
        runningTransform = runningTransform.translate(NECK_TO_CAMERA);
        //Rotate to set x to camera vector
        runningTransform = runningTransform.rotateY(RobotKinematicModel::Head::CAMERA_DECLINATION_ANGLE_OFFSET);
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
    inline std::map<messages::input::ServoID, utility::math::matrix::Transform3D> calculateLegJointPosition(const messages::input::Sensors& sensors, messages::input::ServoID servoID, Side isLeft){
        std::map<messages::input::ServoID, utility::math::matrix::Transform3D> positions;
        utility::math::matrix::Transform3D runningTransform;
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

        //Hip pitch
        runningTransform = runningTransform.translate({
            RobotKinematicModel::Leg::HIP_OFFSET_X,
            negativeIfRight * RobotKinematicModel::Leg::HIP_OFFSET_Y,
            -RobotKinematicModel::Leg::HIP_OFFSET_Z
        });
        //Rotate to face down the leg (see above for definitions of terms, including 'facing')
        runningTransform = runningTransform.rotateY(M_PI_2);
        //Using right hand rule along global z gives positive direction of yaw:
        runningTransform = runningTransform.rotateX(-sensors.servos[static_cast<int>(HIP_YAW)].presentPosition);
        //Return basis facing from body to hip centre (down) with z aligned with the axis of the hip roll motor axis. Position at hip joint
        positions[HIP_YAW] = runningTransform;
        if(servoID == HIP_YAW) {
            return positions;
        }

        runningTransform = runningTransform.rotateZ(sensors.servos[static_cast<int>(HIP_ROLL)].presentPosition);
        //Return basis facing down leg plane, with z oriented through axis of roll motor. Position still hip joint
        positions[HIP_ROLL] = runningTransform;
        if(servoID == HIP_ROLL) {
            return positions;
        }

        //Rotate to face down upper leg
        runningTransform = runningTransform.rotateY(sensors.servos[static_cast<int>(HIP_PITCH)].presentPosition);
        //Translate down upper leg
        runningTransform = runningTransform.translateX(RobotKinematicModel::Leg::UPPER_LEG_LENGTH);
        //Return basis faces down upper leg, with z out of front of thigh. Pos = knee axis centre
        positions[HIP_PITCH] = runningTransform;
        if(servoID == HIP_PITCH) {
            return positions;
        }


        //Rotate to face down lower leg
        runningTransform = runningTransform.rotateY(sensors.servos[static_cast<int>(KNEE)].presentPosition);
        //Translate down lower leg
        runningTransform = runningTransform.translateX(RobotKinematicModel::Leg::UPPER_LEG_LENGTH);
        //Return basis facing down lower leg, with z out of front of shin. Pos = ankle axis centre
        positions[KNEE] = runningTransform;
        if(servoID == KNEE) {
            return positions;
        }


        //Rotate to face down foot (pitch)
        runningTransform = runningTransform.rotateY(sensors.servos[static_cast<int>(ANKLE_PITCH)].presentPosition);
        //Return basis facing pitch down to foot with z out the front of the foot. Pos = ankle axis centre
        positions[ANKLE_PITCH] = runningTransform;
        if(servoID == ANKLE_PITCH) {
            return positions;
        }

        //Rotate to face down foot (roll)
        runningTransform = runningTransform.rotateZ(sensors.servos[static_cast<int>(ANKLE_ROLL)].presentPosition);
        //Rotate so x faces toward toes
        runningTransform = runningTransform.rotateY(-M_PI_2);
        //Translate to ground
        runningTransform = runningTransform.translateZ(-RobotKinematicModel::Leg::FOOT_HEIGHT);
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
    inline std::map<messages::input::ServoID, utility::math::matrix::Transform3D> calculateArmJointPosition(const messages::input::Sensors& sensors, messages::input::ServoID servoID, Side isLeft){
        std::map<messages::input::ServoID, utility::math::matrix::Transform3D> positions;
        utility::math::matrix::Transform3D runningTransform;
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
        runningTransform = runningTransform.translate({
            RobotKinematicModel::Arm::SHOULDER_X_OFFSET,
             negativeIfRight * RobotKinematicModel::Arm::DISTANCE_BETWEEN_SHOULDERS / 2.0,
             RobotKinematicModel::Arm::SHOULDER_Z_OFFSET
         });
        //Rotate about shoulder pitch
        runningTransform = runningTransform.rotateY(sensors.servos[static_cast<int>(SHOULDER_PITCH)].presentPosition);
        //Translate to end of shoulder part
        runningTransform = runningTransform.translate({
            RobotKinematicModel::Arm::SHOULDER_LENGTH,
            negativeIfRight * RobotKinematicModel::Arm::SHOULDER_WIDTH,
            RobotKinematicModel::Arm::SHOULDER_HEIGHT
        });
        //Return matrix pointing down shoulder, y same as global y. Pos = at centre of shoulder roll joint
        positions[SHOULDER_PITCH] = runningTransform;
        if(servoID == SHOULDER_PITCH){
            return positions;
        }

        //Rotate by the shoulder roll
        runningTransform = runningTransform.rotateZ(sensors.servos[static_cast<int>(SHOULDER_ROLL)].presentPosition);
        //Translate to centre of next joint
        runningTransform = runningTransform.translate({
            RobotKinematicModel::Arm::UPPER_ARM_LENGTH,
            negativeIfRight * RobotKinematicModel::Arm::UPPER_ARM_Y_OFFSET,
            RobotKinematicModel::Arm::UPPER_ARM_Z_OFFSET
        });
        positions[SHOULDER_ROLL] = runningTransform;
        if(servoID == SHOULDER_ROLL){
            return positions;
        }

        //Rotate by the elbow angle
        runningTransform = runningTransform.rotateY(sensors.servos[static_cast<int>(ELBOW)].presentPosition);
        //Translate to centre of end of arm, in line with joint
        runningTransform = runningTransform.translate({
            RobotKinematicModel::Arm::LOWER_ARM_LENGTH,
            negativeIfRight * RobotKinematicModel::Arm::LOWER_ARM_Y_OFFSET,
            RobotKinematicModel::Arm::LOWER_ARM_Z_OFFSET
        });
        positions[ELBOW] = runningTransform;
        return positions;
    }

    /*! @brief
    */
    template <typename RobotKinematicModel>
    inline std::map<messages::input::ServoID, utility::math::matrix::Transform3D> calculatePosition(const messages::input::Sensors& sensors, messages::input::ServoID servoID) {
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
        return std::map<messages::input::ServoID, utility::math::matrix::Transform3D>();
    }


    template <typename RobotKinematicModel>
    inline std::map<messages::input::ServoID, utility::math::matrix::Transform3D> calculateAllPositions(const messages::input::Sensors& sensors) {
        std::map<messages::input::ServoID, utility::math::matrix::Transform3D> result = calculatePosition<RobotKinematicModel>(sensors, messages::input::ServoID::L_ANKLE_ROLL);
        std::map<messages::input::ServoID, utility::math::matrix::Transform3D> rightLegPositions = calculatePosition<RobotKinematicModel>(sensors, messages::input::ServoID::R_ANKLE_ROLL);
        std::map<messages::input::ServoID, utility::math::matrix::Transform3D> headPositions = calculatePosition<RobotKinematicModel>(sensors, messages::input::ServoID::HEAD_PITCH);
        std::map<messages::input::ServoID, utility::math::matrix::Transform3D> leftArm = calculatePosition<RobotKinematicModel>(sensors, messages::input::ServoID::L_ELBOW);
        std::map<messages::input::ServoID, utility::math::matrix::Transform3D> rightArm = calculatePosition<RobotKinematicModel>(sensors, messages::input::ServoID::R_ELBOW);
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
    inline arma::vec4 calculateCentreOfMass(const std::map<messages::input::ServoID, utility::math::matrix::Transform3D>& jointPositions, bool includeTorso){
        arma::vec4 totalMassVector = arma::zeros(4);

        for(auto& joint : jointPositions){
            arma::vec4 massVector;
            for(size_t i = 0; i < 4; i++){
                massVector[i] = RobotKinematicModel::MassModel::masses[static_cast<int>(joint.first)][i];
            }
            //NUClear::log<NUClear::DEBUG>("calculateCentreOfMass - reading mass ", messages::input::stringFromId(joint.first), massVector);
            double jointMass = massVector[3];

            utility::math::matrix::Transform3D massScaler;
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
            utility::math::matrix::Transform3D massScaler;
            massScaler.submat(0,0,2,2) *= jointMass;
            totalMassVector +=  massScaler * massVector; // = m * local centre of mass in global robot coords
        }

        utility::math::matrix::Transform3D normaliser;
        if(totalMassVector[3] > 0){
            normaliser.submat(0,0,2,2) *= 1 / totalMassVector[3];
            return normaliser * totalMassVector;
        } else {
            NUClear::log<NUClear::ERROR>("ForwardKinematics::calculateCentreOfMass - Empty centre of mass request or no mass in mass model.");
            return arma::vec4();
        }
    }

    inline utility::math::geometry::Line calculateHorizon(const math::matrix::Rotation3D groundToCamRotation, double cameraDistancePixels){
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

    inline utility::math::matrix::Transform3D calculateBodyToGround(arma::vec3 groundNormal_body, double bodyHeight){
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

        utility::math::matrix::Transform3D groundToBody;
        groundToBody.submat(0,0,2,0) = groundMatrixX;
        groundToBody.submat(0,1,2,1) = groundMatrixY;
        groundToBody.submat(0,2,2,2) = groundNormal_body;
        groundToBody.submat(0,3,2,3) = arma::vec{0, 0, -bodyHeight};
        return groundToBody.i();
    }

    inline arma::mat22 calculateRobotToIMU(math::matrix::Rotation3D orientation) {
        arma::vec3 xRobotImu = orientation.i().col(0);
        arma::vec2 projXRobot = arma::normalise(xRobotImu.rows(0,1));
        arma::vec2 projYRobot = arma::vec2({-projXRobot(1), projXRobot(0)});

        arma::mat22 robotToImu;
        robotToImu.col(0) = projXRobot;
        robotToImu.col(1) = projYRobot;

        return robotToImu;
    }

    template <typename RobotKinematicModel>
    inline arma::vec4 fsrCentreToBody(const messages::input::Sensors& sensors, const arma::vec2& foot, bool left) {
        //sensors.orientationBodyToGround

        int negativeIfRight = left ? 1 : -1;

        arma::vec2 position = foot % arma::vec2({RobotKinematicModel::Leg::FOOT_LENGTH / 2, RobotKinematicModel::Leg::FOOT_WIDTH / 2});
        arma::vec4 centerFoot = arma::vec4({position[0], position[1] + negativeIfRight * RobotKinematicModel::Leg::FOOT_CENTRE_TO_ANKLE_CENTRE, 0, 1});

        return centerFoot;
        // return sensors.forwardKinematics.find(left ? messages::input::ServoID::L_ANKLE_ROLL : messages::input::ServoID::R_ANKLE_ROLL)->second * centerFoot;

    }

    // template <typename RobotKinematicModel>
    // inline arma::vec3 calculateCentreOfPressure(const messages::input::Sensors& sensors) {

    //     sensors.leftFSRCenter
    //     sensors.rightFSRCenter

    //     if (sensors.leftFootDown && sensors.rightFootDown) {

    //     }


    // }

}  // kinematics
}  // motion
}  // utility

#endif  // UTILITY_MOTION_FORWARDKINEMATICS_H
