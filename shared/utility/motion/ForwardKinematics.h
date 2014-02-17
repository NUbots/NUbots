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

#include "messages/input/Sensors.h"
#include "messages/input/ServoID.h"

namespace utility {
namespace motion {
namespace kinematics {   
	

    template <typename RobotKinematicModel>
    arma::mat44 calculateHeadJointPosition(const messages::input::Sensors& sensors, messages::input::ServoID servoID){
        return arma::eye(4,4);
    }
    
    template <typename RobotKinematicModel>
    arma::mat44 calculateArmJointPosition(const messages::input::Sensors& sensors,  messages::input::ServoID servoID, Side isLeft){
        return arma::eye(4,4);
    }

    /*! @brief
        @NOTE read " runningTransform *= utility::math::matrix::_RotationMatrix(angle, 4).t(); " as "Rotate the running transform about its local _ coordinate by angle."
        @return Returns basis matrix for position of end of limb controlled by the specified motor. 

        The basis 'faces' down its x axis.
    */
    template <typename RobotKinematicModel>
    arma::mat44 calculateLegJointPosition(const messages::input::Sensors& sensors, messages::input::ServoID servoID, Side isLeft){
        arma::mat44 runningTransform = arma::eye(4,4);
        //Variables to mask left and right leg differences:
        messages::input::ServoID HIP_YAW, HIP_ROLL, HIP_PITCH, KNEE, ANKLE_PITCH, ANKLE_ROLL;
        int MIRROR_FACTOR_HIP_YAW(1),
            MIRROR_FACTOR_HIP_ROLL(1),
            MIRROR_FACTOR_HIP_PITCH(1),
            MIRROR_FACTOR_KNEE(1),
            MIRROR_FACTOR_ANKLE_PITCH(1),
            MIRROR_FACTOR_ANKLE_ROLL(1);
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
            MIRROR_FACTOR_HIP_YAW = RobotKinematicModel::Leg::LEFT_TO_RIGHT_HIP_YAW;
            MIRROR_FACTOR_HIP_ROLL = RobotKinematicModel::Leg::LEFT_TO_RIGHT_HIP_ROLL;
            MIRROR_FACTOR_HIP_PITCH = RobotKinematicModel::Leg::LEFT_TO_RIGHT_HIP_PITCH;
            MIRROR_FACTOR_KNEE = RobotKinematicModel::Leg::LEFT_TO_RIGHT_KNEE;
            MIRROR_FACTOR_ANKLE_PITCH = RobotKinematicModel::Leg::LEFT_TO_RIGHT_ANKLE_PITCH;
            MIRROR_FACTOR_ANKLE_ROLL = RobotKinematicModel::Leg::LEFT_TO_RIGHT_ANKLE_ROLL;
            negativeIfRight = -1;
        }
        
        arma::mat44 hipPos = arma::eye(4,4);
        hipPos.col(3)= arma::vec({ RobotKinematicModel::Leg::HIP_OFFSET_X, negativeIfRight * RobotKinematicModel::Leg::LENGTH_BETWEEN_LEGS/2, -RobotKinematicModel::Leg::HIP_OFFSET_Z,1});
        runningTransform *= hipPos;
        //Rotate to face down the leg (see above for definitions of terms, including 'facing')
        runningTransform *= utility::math::matrix::yRotationMatrix(M_PI/2, 4).t();
        //Using right hand rule along global z gives positive direction of yaw:
        runningTransform *= utility::math::matrix::xRotationMatrix(-sensors.servos[static_cast<int>(HIP_YAW)].presentPosition * MIRROR_FACTOR_HIP_YAW, 4).t();
        //Return basis facing from body to hip centre (down) with z aligned with the axis of the hip roll motor axis. Position at hip joint
        if(servoID == messages::input::ServoID::L_HIP_YAW || servoID == messages::input::ServoID::R_HIP_YAW) { 
            return runningTransform; 
        }
        std::cout << "HIP_YAW \n" << runningTransform <<std::endl; 


        runningTransform *= utility::math::matrix::zRotationMatrix(sensors.servos[static_cast<int>(HIP_ROLL)].presentPosition * MIRROR_FACTOR_HIP_ROLL, 4).t();
        //Return basis facing down leg plane, with z oriented through axis of roll motor. Position still hip joint
        if(servoID == messages::input::ServoID::L_HIP_ROLL || servoID == messages::input::ServoID::R_HIP_ROLL) { 
            return runningTransform;
        }
        std::cout << "HIP_ROLL \n" << runningTransform <<std::endl; 


        //Rotate to face down upper leg
        runningTransform *= utility::math::matrix::yRotationMatrix(sensors.servos[static_cast<int>(HIP_PITCH)].presentPosition * MIRROR_FACTOR_HIP_PITCH, 4).t();
        //Translate down upper leg
        runningTransform *= utility::math::matrix::translationMatrix(arma::vec3({RobotKinematicModel::Leg::UPPER_LEG_LENGTH, 0, 0}));
        //Return basis faces down upper leg, with z out of front of thigh. Pos = knee axis centre
        if(servoID == messages::input::ServoID::L_HIP_PITCH || servoID == messages::input::ServoID::R_HIP_PITCH) { 
            return runningTransform; 
        }
        std::cout << "HIP_PITCH \n" << runningTransform <<std::endl; 


        //Rotate to face down lower leg
        runningTransform *= utility::math::matrix::yRotationMatrix(sensors.servos[static_cast<int>(KNEE)].presentPosition * MIRROR_FACTOR_KNEE, 4).t();
        //Translate down lower leg
        runningTransform *= utility::math::matrix::translationMatrix(arma::vec3({RobotKinematicModel::Leg::UPPER_LEG_LENGTH, 0, 0}));
        //Return basis facing down lower leg, with z out of front of shin. Pos = ankle axis centre
        if(servoID == messages::input::ServoID::L_KNEE || servoID == messages::input::ServoID::R_KNEE) { 
            return runningTransform; 
        }
        std::cout << "KNEE \n" << runningTransform <<std::endl; 


        //Rotate to face down foot (pitch)
        runningTransform *= utility::math::matrix::yRotationMatrix(sensors.servos[static_cast<int>(ANKLE_PITCH)].presentPosition * MIRROR_FACTOR_ANKLE_PITCH, 4).t();
        //Return basis facing pitch down to foot with z out the front of the foot. Pos = ankle axis centre
        if(servoID == messages::input::ServoID::L_ANKLE_PITCH || servoID == messages::input::ServoID::R_ANKLE_PITCH) { 
            return runningTransform; 
        }
        std::cout << "ANKLE_PITCH \n" << runningTransform <<std::endl; 

        //Rotate to face down foot (roll)
        runningTransform *= utility::math::matrix::zRotationMatrix(sensors.servos[static_cast<int>(ANKLE_ROLL)].presentPosition * MIRROR_FACTOR_ANKLE_ROLL, 4).t();
        //Return basis with x as the normal the plane of the foot and z out the front. Pos = ankle axis centre
        return runningTransform;       
    }

    /*! @brief 
    */
    template <typename RobotKinematicModel>
    arma::mat44 calculatePosition(const messages::input::Sensors& sensors, messages::input::ServoID servoID) {
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
        return arma::eye(4,4);
    }
   

} // kinematics
}  // motion
}  // utility

#endif  // UTILITY_MOTION_FORWARDKINEMATICS_H