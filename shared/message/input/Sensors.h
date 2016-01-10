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

#ifndef MESSAGE_INPUT_SENSORS_H
#define MESSAGE_INPUT_SENSORS_H

#include <nuclear>
#include <armadillo>
#include <nuclear>

#include "ServoID.h"

#include "utility/math/matrix/Transform3D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/geometry/Line.h"

namespace message {
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

            struct Button {
                uint id;
                bool value;
            };

            struct LED {
                uint id;
                uint32_t colour;
            };

            NUClear::clock::time_point timestamp;

            float voltage;
            float battery;
            arma::vec3 accelerometer;
            arma::vec3 gyroscope;

            /*! The orientation matrix is the map from world to robot coordinates, measured by the gyro. It is the world coordinates in columns relative to the robot.*/
            utility::math::matrix::Rotation3D orientation;

            std::vector<Servo> servos;
            std::vector<Button> buttons;
            std::vector<LED> leds;

            arma::vec3 centreOfPressure;
            arma::vec2 leftFSRCenter;
            arma::vec2 rightFSRCenter;

            arma::mat22 robotToIMU;

            bool leftFootDown;
            bool rightFootDown;

            std::map<message::input::ServoID, utility::math::matrix::Transform3D> forwardKinematics;

            // utility::math::Transform3D odometry;
            arma::vec2 odometry;
            arma::mat22 odometryCovariance;

            float bodyCentreHeight;

            arma::vec4 centreOfMass;

            utility::math::matrix::Transform3D orientationBodyToGround;
            utility::math::matrix::Transform3D orientationCamToGround;
            utility::math::matrix::Transform3D kinematicsBodyToGround;
            utility::math::matrix::Transform3D kinematicsCamToGround;

        };
    }
}

#endif
