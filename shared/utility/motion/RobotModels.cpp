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

#include "RobotModels.h"

namespace utility{
    namespace motion{
        namespace kinematics {
            //Convention: all values positive
            
            float DarwinModel::Leg::HIP_OFFSET_X = 0.00;
            float DarwinModel::Leg::HIP_OFFSET_Y = 0.037; //DARWIN SAYS THIS IS 0.008
            float DarwinModel::Leg::HIP_OFFSET_Z = 0.034;
            float DarwinModel::Leg::UPPER_LEG_LENGTH = 0.093;
            float DarwinModel::Leg::LOWER_LEG_LENGTH = 0.093;
            float DarwinModel::Leg::FOOT_HEIGHT = 0.0335;

          	float DarwinModel::Leg::FOOT_LENGTH = 0.094; // rough
            float DarwinModel::Leg::TOE_LENGTH = 0.0472; //measured
            float DarwinModel::Leg::HEEL_LENGTH = 0.0451; //measured

            float DarwinModel::Leg::FOOT_WIDTH = 0.066; // rough
            float DarwinModel::Leg::FOOT_CENTRE_TO_ANKLE_CENTRE = 0.011; // rough

            float DarwinModel::Leg::LENGTH_BETWEEN_LEGS = DarwinModel::Leg::HIP_OFFSET_Y * 2;

            int DarwinModel::Leg::LEFT_TO_RIGHT_HIP_YAW =       -1;
            int DarwinModel::Leg::LEFT_TO_RIGHT_HIP_ROLL =      -1;
            int DarwinModel::Leg::LEFT_TO_RIGHT_HIP_PITCH =      1;
            int DarwinModel::Leg::LEFT_TO_RIGHT_KNEE =           1;
            int DarwinModel::Leg::LEFT_TO_RIGHT_ANKLE_PITCH =    1;
            int DarwinModel::Leg::LEFT_TO_RIGHT_ANKLE_ROLL =    -1;

            float DarwinModel::Head::NECK_BASE_POS_FROM_ORIGIN_X = 0.013;
            float DarwinModel::Head::NECK_BASE_POS_FROM_ORIGIN_Y = 0;
            float DarwinModel::Head::NECK_BASE_POS_FROM_ORIGIN_Z = 0.11;
            float DarwinModel::Head::NECK_LENGTH = 0.042;
            float DarwinModel::Head::NECK_TO_CAMERA_X = 0.036;
            float DarwinModel::Head::NECK_TO_CAMERA_Y = 0;
            float DarwinModel::Head::NECK_TO_CAMERA_Z = 0.028;
            float DarwinModel::Head::CAMERA_DECLINATION_ANGLE_OFFSET = 0;// 0.162; default zero
            //Head movement limits
            float DarwinModel::Head::MAX_YAW = M_PI * 2 / 3;
            float DarwinModel::Head::MIN_YAW = -M_PI * 2 / 3;
            float DarwinModel::Head::MAX_PITCH = M_PI / 3;
            float DarwinModel::Head::MIN_PITCH = -M_PI / 3;

            float DarwinModel::Arm::DISTANCE_BETWEEN_SHOULDERS = 0.114;
            float DarwinModel::Arm::SHOULDER_Z_OFFSET = 0.088;
            float DarwinModel::Arm::SHOULDER_X_OFFSET = 0.01;

            float DarwinModel::Arm::SHOULDER_LENGTH = 0.00;
            float DarwinModel::Arm::SHOULDER_WIDTH = 0.0245;
            float DarwinModel::Arm::SHOULDER_HEIGHT = 0.017;

            float DarwinModel::Arm::UPPER_ARM_LENGTH = 0.0615;
            float DarwinModel::Arm::UPPER_ARM_Y_OFFSET = 0;
            float DarwinModel::Arm::UPPER_ARM_X_OFFSET = 0.02;   //Very rough

            float DarwinModel::Arm::LOWER_ARM_LENGTH = 0.13;
            float DarwinModel::Arm::LOWER_ARM_Y_OFFSET = 0;
            float DarwinModel::Arm::LOWER_ARM_Z_OFFSET = 0;  //Very rough

            std::array<arma::vec4, 21> DarwinModel::MassModel::masses = {
                arma::vec4({-0.011264,         0.0109774,      -0.00139357,    0.025913}),  //  R_SHOULDER_PITCH
                arma::vec4({-0.011264,         -0.0109774,     -0.00139357,    0.025913}),  //  L_SHOULDER_PITCH
                arma::vec4({-0.025261,         -0.000659787,   0.000734065,    0.168377}),  //  R_SHOULDER_ROLL
                arma::vec4({-0.025261,         0.000659787,    0.000734065,    0.168377}),  //  L_SHOULDER_ROLL
                arma::vec4({-0.0841618,        -0.00666564,    -0.0134901,     0.0592885}), //  R_ELBOW
                arma::vec4({-0.0841618,        0.00666564,     -0.0134901,     0.0592885}), //  L_ELBOW
                arma::vec4({-0.0155628,        0,              0.000480135,    0.0270692}), //  R_HIP_YAW
                arma::vec4({-0.0155628,        0,              0.000480135,    0.0270692}), //  L_HIP_YAW
                arma::vec4({0.0138731,         -7.99828e-005,  -0.0182424,     0.167108}),  //  R_HIP_ROLL
                arma::vec4({0.0138731,         7.99828e-005,   -0.0182424,     0.167108}),  //  L_HIP_ROLL
                arma::vec4({-0.0300345,        0.000322635,    0.000691906,    0.119043}),  //  R_HIP_PITCH
                arma::vec4({-0.0300345,        -0.000322635,   0.000691906,    0.119043}),  //  L_HIP_PITCH
                arma::vec4({-0.0539545,        0.000592469,    0.00654763,     0.0703098}), //  R_KNEE
                arma::vec4({-0.0539545,        -0.000592469,   0.00654763,     0.0703098}), //  L_KNEE
                arma::vec4({-0.0138731,        0.000213732,    -0.0185361,     0.167108}),  //  R_ANKLE_PITCH
                arma::vec4({-0.0138731,        -0.000213732,   -0.0185361,     0.167108}),  //  L_ANKLE_PITCH
                arma::vec4({0.0259953,         -0.00950588,    -0.000502877,   0.0794462}), //  R_ANKLE_ROLL
                arma::vec4({0.0259953,         0.00950588,     -0.000502877,   0.0794462}), //  L_ANKLE_ROLL
                arma::vec4({-0.0165676,        0.00142428,     0.000712811,    0.0243577}), //  HEAD_YAW
                arma::vec4({-0.035,                     0,           0.01,      0.11708}),  //  HEAD_PITCH
                arma::vec4({-0.0066631,        -0.00311589,      0.0705563,      0.975599}) //  TORSO
            };

            float DarwinModel::TEAMDARWINCHEST_TO_ORIGIN = 0.096 - DarwinModel::Leg::HIP_OFFSET_Z; //Taken from team darwin OPkinematics.cpp : hipOffsetZ = .096;

        }
    }
}