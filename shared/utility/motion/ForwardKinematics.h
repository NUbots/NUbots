/*
 * This file is part of InverseKinematics.
 *
 * InverseKinematics is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * InverseKinematics is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with InverseKinematics.  If not, see <http://www.gnu.org/licenses/>.
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

#include "messages/input/ServoID.h"

namespace utility {
namespace motion {
namespace kinematics {   
	/*! @brief 
	*/
    template <typename RobotKinematicModel>
    arma::mat44 calculatePosition(const Sensors& sensors, messages::input::ServoID servoID) {
        switch(target) {
            case HEAD_YAW:
            case HEAD_PITCH:
                return calculateHeadJointPosition<RobotKinematicModel>(sensors, servoID);
            case R_SHOULDER_PITCH:
            case R_SHOULDER_ROLL:
            case R_ELBOW:
                return calculateArmJointPosition<RobotKinematicModel>(sensors, servoID, Side::RIGHT);
            case L_SHOULDER_PITCH:
            case L_SHOULDER_ROLL:
            case L_ELBOW:
                return calculateArmJointPosition<RobotKinematicModel>(sensors, servoID, Side::LEFT);
            case R_HIP_YAW:
            case R_HIP_ROLL:
            case R_HIP_PITCH:
            case R_KNEE:
            case R_ANKLE_PITCH:
            case R_ANKLE_ROLL:
                return calculateLegJointPosition<RobotKinematicModel>(sensors, servoID, Side::RIGHT);
            case L_HIP_YAW:
            case L_HIP_ROLL:
            case L_HIP_PITCH:
            case L_KNEE:
            case L_ANKLE_PITCH:
            case L_ANKLE_ROLL:
                return calculateLegJointPosition<RobotKinematicModel>(sensors, servoID, Side::LEFT);
            case default:
                return arma::eye(4,4);
        }
    }

    template <typename RobotKinematicModel>
    arma::mat44 calculateHeadJointPosition(const Sensors& sensors, messages::input::ServoID servoID){

    }
    
    template <typename RobotKinematicModel>
    arma::mat44 calculateArmJointPosition(const Sensors& sensors,  messages::input::ServoID servoID, Side isLeft){
          
    }

    template <typename RobotKinematicModel>
    arma::mat44 calculateLegJointPosition(const Sensors& sensors, messages::input::ServoID servoID, Side isLeft){
        arma::mat44 runningTransform = arma::eye(4,4);

        arma::mat44 hipPos = arma::eye(4,4);
        hipPos.col(3) = arma::vec({ 0, (isLeft ? 1 : -1) * RobotKinematicModel::Leg::LENGTH_BETWEEN_LEGS/2, -RobotKinematicModel::Leg::DISTANCE_FROM_BODY_TO_HIP_JOINT});

        runningTransform *= hipPos;
        runningTransform = utility::math::yRotationMatrix(M_PI/2, 4) * runningTransform;
        runningTransform = utility::math::xRotationMatrix(-sensors.servo[isLeft ? ServoID::L_HIP_YAW : ServoID::R_HIP_YAW], 4) * runningTransform;

        //if(hipyaw) return runningTransform;

        
    }

   

} // kinematics
}  // motion
}  // utility

#endif  // UTILITY_MOTION_FORWARDKINEMATICS_H