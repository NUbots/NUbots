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
#include <array>

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
                    static float HIP_OFFSET_X;
                    static float HIP_OFFSET_Y;
                    static float HIP_OFFSET_Z;
                    static float UPPER_LEG_LENGTH;
                    static float LOWER_LEG_LENGTH;
                    static float FOOT_HEIGHT;

                    static float FOOT_LENGTH;
                    static float TOE_LENGTH;
                    static float HEEL_LENGTH;

                    static float FOOT_WIDTH;
                    static float FOOT_CENTRE_TO_ANKLE_CENTRE;

                    static float LENGTH_BETWEEN_LEGS;

                    static int LEFT_TO_RIGHT_HIP_YAW;
                    static int LEFT_TO_RIGHT_HIP_ROLL;
                    static int LEFT_TO_RIGHT_HIP_PITCH;
                    static int LEFT_TO_RIGHT_KNEE;
                    static int LEFT_TO_RIGHT_ANKLE_PITCH;
                    static int LEFT_TO_RIGHT_ANKLE_ROLL;
                };

                class Head {
                public:
                    static float NECK_BASE_POS_FROM_ORIGIN_X;
                    static float NECK_BASE_POS_FROM_ORIGIN_Y;
                    static float NECK_BASE_POS_FROM_ORIGIN_Z;
                    static float NECK_LENGTH;
                    static float NECK_TO_CAMERA_X;
                    static float NECK_TO_CAMERA_Y;
                    static float NECK_TO_CAMERA_Z;
                    static float CAMERA_DECLINATION_ANGLE_OFFSET;
                    //Head movement limits
                    static float MAX_YAW;
                    static float MIN_YAW;
                    static float MAX_PITCH;
                    static float MIN_PITCH;
                };

                // ROUGH MEASUREMENTS
                class Arm {
                public:
                    static float DISTANCE_BETWEEN_SHOULDERS;
                    static float SHOULDER_Z_OFFSET;
                    static float SHOULDER_X_OFFSET;

                    static float SHOULDER_LENGTH;
                    static float SHOULDER_WIDTH;
                    static float SHOULDER_HEIGHT;

                    static float UPPER_ARM_LENGTH;
                    static float UPPER_ARM_Y_OFFSET;
                    static float UPPER_ARM_X_OFFSET;

                    static float LOWER_ARM_LENGTH;
                    static float LOWER_ARM_Y_OFFSET;
                    static float LOWER_ARM_Z_OFFSET;


                };

                class MassModel {
                public:
                    static std::array<arma::vec4, 21> masses;

                };

                static float TEAMDARWINCHEST_TO_ORIGIN;
            };

        }
    }
}

#endif
