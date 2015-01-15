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

#ifndef MESSAGES_INPUT_SENSORS_H
#define MESSAGES_INPUT_SENSORS_H

#include <nuclear>
#include <armadillo>
#include <nuclear>

#include "ServoID.h"

#include "utility/math/matrix/Transform3D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/geometry/Line.h"

namespace messages {
    namespace input {


        struct Sensors {
            struct Servo {
                uint16_t errorFlags;

                ServoID id;

                bool enabled;

                float pGain;
                float iGain;
                float dGain;

                float goalPosition;
                float goalVelocity;

                float presentPosition;
                float presentVelocity;

                float load;
                float voltage;
                float temperature;
            };

            NUClear::clock::time_point timestamp;

            arma::vec3 accelerometer;
            arma::vec3 gyroscope;
            utility::math::matrix::Rotation3D orientation;
            arma::vec3 leftFSR;
            arma::vec3 rightFSR;

            arma::mat22 robotToIMU;

            bool leftFootDown;
            bool rightFootDown;

            std::map<messages::input::ServoID, utility::math::matrix::Transform3D> forwardKinematics;

            // utility::math::Transform3D odometry;
            arma::vec2 odometry;
            arma::mat22 odometryCovariance;

            float bodyCentreHeight;

            arma::vec4 centreOfMass;

            utility::math::matrix::Transform3D orientationBodyToGround;
            utility::math::matrix::Transform3D orientationCamToGround;
            utility::math::matrix::Transform3D kinematicsBodyToGround;
            utility::math::matrix::Transform3D kinematicsCamToGround;

            std::vector<Servo> servos;
        };
    }
}

#endif