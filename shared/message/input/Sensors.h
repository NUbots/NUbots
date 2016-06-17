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

            struct FSR {
                arma::vec2 centre;
                std::vector<double> values;
            };

            NUClear::clock::time_point timestamp;

            float voltage;
            float battery;
            arma::vec3 accelerometer;
            arma::vec3 gyroscope;

            /// This is the transform from the robots space to world space. It measures the world in robot space.
            utility::math::matrix::Transform3D world;

            std::vector<Servo> servos;
            std::vector<Button> buttons;
            std::vector<LED> leds;
            std::vector<FSR> fsrs;

            arma::vec3 centreOfPressure;

            arma::mat22 robotToIMU;

            /// Percentage of the left foot that's considered "down" i.e. if 3/4 FSR sensors have weight this is 0.75
            bool leftFootDown;
            bool rightFootDown;

            std::map<message::input::ServoID, utility::math::matrix::Transform3D> forwardKinematics;

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
