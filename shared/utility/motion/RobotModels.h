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

#ifndef UTILITY_MOTION_KINEMATICS_ROBOTMODELS_H
#define UTILITY_MOTION_KINEMATICS_ROBOTMODELS_H

#include <armadillo>

namespace utility{
    namespace motion{
        namespace kinematics{

            enum class Side : bool {
                LEFT = true,
                RIGHT = false
            };

            class DarwinModel {
            public:

                class Leg {
                public:
                    static constexpr float HIP_OFFSET_X = 0.00; //DARWIN SAYS THIS IS 0.008
                    static constexpr float HIP_OFFSET_Y = 0.037; //DARWIN SAYS THIS IS 0.008
                    static constexpr float HIP_OFFSET_Z = 0.034;
                    static constexpr float UPPER_LEG_LENGTH = 0.093;
                    static constexpr float LOWER_LEG_LENGTH = 0.093;
                    static constexpr float FOOT_HEIGHT = 0.0335;
                    static constexpr float FOOT_LENGTH = 0.105; // rough
                    static constexpr float FOOT_WIDTH = 0.066; // rough
                    static constexpr float FOOT_CENTRE_TO_ANKLE_CENTRE = 0.011; // rough

                    static constexpr float LENGTH_BETWEEN_LEGS = HIP_OFFSET_Y * 2;

                    static constexpr int LEFT_TO_RIGHT_HIP_YAW =       -1;
                    static constexpr int LEFT_TO_RIGHT_HIP_ROLL =      -1;
                    static constexpr int LEFT_TO_RIGHT_HIP_PITCH =      1;
                    static constexpr int LEFT_TO_RIGHT_KNEE =           1;
                    static constexpr int LEFT_TO_RIGHT_ANKLE_PITCH =    1;
                    static constexpr int LEFT_TO_RIGHT_ANKLE_ROLL =    -1;
                };

                class Head {
                public:
                    static constexpr float NECK_BASE_POS_FROM_ORIGIN_X = 0.013;
                    static constexpr float NECK_BASE_POS_FROM_ORIGIN_Y = 0;
                    static constexpr float NECK_BASE_POS_FROM_ORIGIN_Z = 0.11;
                    static constexpr float NECK_LENGTH = 0.042;
                    static constexpr float NECK_TO_CAMERA_X = 0.023;
                    static constexpr float NECK_TO_CAMERA_Y = 0;
                    static constexpr float NECK_TO_CAMERA_Z = 0.028
                    static constexpr float CAMERA_DECLINATION_ANGLE_OFFSET = 0;
                    //Head movement limits
                    static constexpr float MAX_YAW = M_PI * 2 / 3;
                    static constexpr float MIN_YAW = -M_PI * 2 / 3;
                    static constexpr float MAX_PITCH = M_PI / 3;
                    static constexpr float MIN_PITCH = -M_PI / 3;
                };

                class Arm {//ROUGH MEASUREMENTS
                public:
                    static constexpr float DISTANCE_BETWEEN_SHOULDERS = 0.114;
                    static constexpr float SHOULDER_Z_OFFSET = 0.088;
                    static constexpr float SHOULDER_X_OFFSET = 0.01;

                    static constexpr float SHOULDER_LENGTH = 0.017;
                    static constexpr float SHOULDER_WIDTH = 0.0245;
                    static constexpr float SHOULDER_HEIGHT = 0;

                    static constexpr float UPPER_ARM_LENGTH = 0.0615;
                    static constexpr float UPPER_ARM_Y_OFFSET = 0;
                    static constexpr float UPPER_ARM_Z_OFFSET = 0.02;   //Very rough

                    static constexpr float LOWER_ARM_LENGTH = 0.13;
                    static constexpr float LOWER_ARM_Y_OFFSET = 0;
                    static constexpr float LOWER_ARM_Z_OFFSET = 0;  //Very rough


                };

                class MassModel {
                public:
                    static constexpr size_t NUMBER_OF_MASSES = 21;
                    static constexpr size_t MASS_REPRESENTATION_DIMENSION = 4;
                    static constexpr double masses[NUMBER_OF_MASSES][MASS_REPRESENTATION_DIMENSION] = {
                        {-0.011264,         0.0109774,      -0.00139357,    0.025913},  //  R_SHOULDER_PITCH
                        {-0.011264,         -0.0109774,     -0.00139357,    0.025913},  //  L_SHOULDER_PITCH
                        {-0.025261,         -0.000659787,   0.000734065,    0.168377},  //  R_SHOULDER_ROLL
                        {-0.025261,         0.000659787,    0.000734065,    0.168377},  //  L_SHOULDER_ROLL
                        {-0.0841618,        -0.00666564,    -0.0134901,     0.0592885}, //  R_ELBOW
                        {-0.0841618,        0.00666564,     -0.0134901,     0.0592885}, //  L_ELBOW
                        {-0.0155628,        0,              0.000480135,    0.0270692}, //  R_HIP_YAW
                        {-0.0155628,        0,              0.000480135,    0.0270692}, //  L_HIP_YAW
                        {0.0138731,         -7.99828e-005,  -0.0182424,     0.167108},  //  R_HIP_ROLL
                        {0.0138731,         7.99828e-005,   -0.0182424,     0.167108},  //  L_HIP_ROLL
                        {-0.0300345,        0.000322635,    0.000691906,    0.119043},  //  R_HIP_PITCH
                        {-0.0300345,        -0.000322635,   0.000691906,    0.119043},  //  L_HIP_PITCH
                        {-0.0539545,        0.000592469,    0.00654763,     0.0703098}, //  R_KNEE
                        {-0.0539545,        -0.000592469,   0.00654763,     0.0703098}, //  L_KNEE
                        {-0.0138731,        0.000213732,    -0.0185361,     0.167108},  //  R_ANKLE_PITCH
                        {-0.0138731,        -0.000213732,   -0.0185361,     0.167108},  //  L_ANKLE_PITCH
                        {0.0259953,         -0.00950588,    -0.000502877,   0.0794462}, //  R_ANKLE_ROLL
                        {0.0259953,         0.00950588,     -0.000502877,   0.0794462}, //  L_ANKLE_ROLL
                        {-0.0165676,        0.00142428,     0.000712811,    0.0243577}, //  HEAD_YAW
                        {-0.02533338,       6.39198E-005,   -0.0144355,     0.158042},  //  HEAD_PITCH
                        {-0.0066631,        -0.00311589,    0.0705563,      0.975599}   //  TORSO
                    };

                };
                static constexpr float TEAMDARWINCHEST_TO_ORIGIN = 0.096 - Leg::HIP_OFFSET_Z; //Taken from team darwin OPkinematics.cpp : hipOffsetZ = .096;
            };

        }
    }
}

#endif
