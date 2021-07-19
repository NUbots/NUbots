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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch.hpp>

#include "message/input/Sensors.hpp"
#include "message/motion/BodySide.hpp"
#include "message/motion/KinematicsModel.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/motion/ForwardKinematics.hpp"
#include "utility/motion/InverseKinematics.hpp"

using message::input::Sensors;
using message::motion::BodySide;
using message::motion::KinematicsModel;

using utility::input::LimbID;
using utility::input::ServoID;

static const KinematicsModel kinematics_model = {
    // Leg
    {0.0f, 0.055f, 0.045f, 0.2f, 0.2f, 0.04f, 0.215f, 0.13f, 0.085f, 0.13f, 0.02f, 2.0f * 0.055f, -1, -1, 1, 1, 1, -1},
    // Head
    {-0.007f,
     0.0f,
     0.21f,
     0.048f,
     0.069f,
     0.0f,
     0.065f,
     float(M_PI) / 90.0f,
     0.068f,
     -float(M_PI_4),
     float(M_PI_4),
     -float(M_PI) / 6.0f,
     float(M_PI) / 6.0f}};

static constexpr double ITERATIONS      = 10000;
static constexpr double ERROR_THRESHOLD = 2e-5;


TEST_CASE("Test the Head kinematics", "[utility][motion][kinematics][head]") {
    for (int i = 0; i < ITERATIONS; ++i) {

        // Make a random camera vector
        Eigen::Vector3d camera_vector = Eigen::Vector3d::Random().normalized();

        INFO("Testing with the random vector, " << camera_vector.transpose());

        std::vector<std::pair<ServoID, double>> angles =
            utility::motion::kinematics::calculateCameraLookJoints(kinematics_model, camera_vector);

        // Make our sensors object
        Sensors sensors;
        sensors.servo = std::vector<Sensors::Servo>(20);

        // Insert our known sensors (from the calculated angles) into our sensors
        for (const auto& angle : angles) {
            const auto [servoID, position]                               = angle;
            sensors.servo[static_cast<size_t>(servoID)].present_position = static_cast<float>(position);
        }

        // Do our forward kinematics
        Eigen::Affine3d Htc = utility::motion::kinematics::calculatePosition(kinematics_model,
                                                                             sensors,
                                                                             ServoID::HEAD_PITCH)[ServoID::HEAD_PITCH];

        // Check that our vector that forward kinematics finds is close to what is expected
        REQUIRE(double(Htc(0, 0) - camera_vector[0]) == Approx(0.0).margin(ERROR_THRESHOLD));
        REQUIRE(double(Htc(1, 0) - camera_vector[1]) == Approx(0.0).margin(ERROR_THRESHOLD));
        REQUIRE(double(Htc(2, 0) - camera_vector[2]) == Approx(0.0).margin(ERROR_THRESHOLD));
    }
}

TEST_CASE("Test the Leg kinematics", "[utility][motion][kinematics][leg]") {
    for (int i = 0; i < ITERATIONS; ++i) {

        // Make a random camera vector
        Eigen::Affine3d ik_request = Eigen::Affine3d::Identity();
        Eigen::Vector3d rotation   = Eigen::Vector3d::Random() * 2.0 * M_PI;
        ik_request                 = ik_request.rotate(Eigen::AngleAxisd(rotation.x(), Eigen::Vector3d::UnitX()));
        ik_request                 = ik_request.rotate(Eigen::AngleAxisd(rotation.y(), Eigen::Vector3d::UnitY()));
        ik_request                 = ik_request.rotate(Eigen::AngleAxisd(rotation.z(), Eigen::Vector3d::UnitZ()));
        ik_request.translation()   = ((Eigen::Vector3d::Random() + Eigen::Vector3d::Ones()) * 0.5)
                                       .cwiseProduct(Eigen::Vector3d(0.03, 0.03, 0.10));

        INFO("Testing with the random transform, \n" << ik_request.matrix());

        Sensors sensors;
        sensors.servo = std::vector<Sensors::Servo>(20);

        std::vector<std::pair<ServoID, double>> left_leg_joints =
            utility::motion::kinematics::calculateLegJoints(kinematics_model, ik_request, LimbID::LEFT_LEG);
        for (const auto& leg_joint : left_leg_joints) {
            const auto [servoID, position]          = leg_joint;
            sensors.servo[servoID].present_position = static_cast<float>(position);
        }

        std::vector<std::pair<ServoID, double>> right_leg_joints =
            utility::motion::kinematics::calculateLegJoints(kinematics_model, ik_request, LimbID::RIGHT_LEG);
        for (const auto& leg_joint : right_leg_joints) {
            const auto [servoID, position]          = leg_joint;
            sensors.servo[servoID].present_position = static_cast<float>(position);
        }

        INFO("Calculating forward kinematics");
        Eigen::Affine3d left_foot_position =
            utility::motion::kinematics::calculatePosition(kinematics_model,
                                                           sensors,
                                                           ServoID::L_ANKLE_ROLL)[ServoID::L_ANKLE_ROLL];
        Eigen::Affine3d right_foot_position =
            utility::motion::kinematics::calculatePosition(kinematics_model,
                                                           sensors,
                                                           ServoID::R_ANKLE_ROLL)[ServoID::R_ANKLE_ROLL];
        INFO("Forward Kinematics predicts left foot: \n" << left_foot_position.matrix());
        INFO("Forward Kinematics predicts right foot: \n" << right_foot_position.matrix());
        INFO("Compared to request: \n" << ik_request.matrix());
        for (size_t servo_id = 0; servo_id < ServoID::NUMBER_OF_SERVOS; ++servo_id) {
            INFO(ServoID(servo_id) << ": " << sensors.servo[servo_id].present_position);
        }

        const double lerror = (left_foot_position.matrix().array() - ik_request.matrix().array()).abs().maxCoeff();
        const double rerror = (right_foot_position.matrix().array() - ik_request.matrix().array()).abs().maxCoeff();

        REQUIRE(lerror == Approx(0.0).margin(ERROR_THRESHOLD));
        REQUIRE(rerror == Approx(0.0).margin(ERROR_THRESHOLD));
    }
}
