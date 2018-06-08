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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include <catch.hpp>

#include "message/input/Sensors.h"
#include "message/input/ServoID.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/motion/InverseKinematics.h"

using message::input::Sensors;
using message::input::ServoID;
using message::motion::kinematics::KinematicsModel;
using utility::motion::kinematics::calculateCameraLookJoints;

TEST_CASE("Test the Head kinematics", "[utility][motion][kinematics][head]") {

    const double ITERATIONS = 10000;

    srand(time(nullptr));

    KinematicsModel kinematicsModel;

    for (int i = 0; i < ITERATIONS; ++i) {

        // Make a random camera vector
        arma::vec3 camVec = {double(rand()), double(rand()), double(rand())};
        camVec            = arma::normalise(camVec);

        INFO("Testing with the random vector, " << camVec.t());

        std::vector<std::pair<message::input::ServoID, float>> angles =
            utility::motion::kinematics::calculateCameraLookJoints(kinematicsModel, camVec);

        // Make our sensors object
        Sensors sensors;
        sensors.servos = std::vector<Sensors::Servo>(20);

        // Insert our known sensors (from the calculated angles) into our sensors
        for (auto& angle : angles) {
            ServoID servoID;
            float position;

            std::tie(servoID, position) = angle;

            sensors.servos[static_cast<int>(servoID)].presentPosition = position;
        }

        // Do our forward kinematics
        arma::mat44 fKin = utility::motion::kinematics::calculatePosition(
            KinematicsModel(), sensors, ServoID::HEAD_PITCH)[ServoID::HEAD_PITCH];

        // Check that our vector that forward kinematics finds is close to what is expected
        REQUIRE(double(fKin(0, 0) - camVec[0]) == Approx(0));
        REQUIRE(double(fKin(1, 0) - camVec[1]) == Approx(0));
        REQUIRE(double(fKin(2, 0) - camVec[2]) == Approx(0));
    }
}
