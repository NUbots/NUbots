/*
 * MIT License
 *
 * Copyright (c) 2019 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "message/actuation/BodySide.hpp"
#include "message/actuation/KinematicsModel.hpp"
#include "message/actuation/Servos.hpp"
#include "message/input/Sensors.hpp"

#include "utility/actuation/ForwardKinematics.hpp"
#include "utility/actuation/InverseKinematics.hpp"
#include "utility/input/LimbID.hpp"

using Catch::Matchers::WithinAbs;
using message::actuation::BodySide;
using message::actuation::KinematicsModel;
using message::actuation::ServoID;
using message::input::Sensors;
using utility::input::LimbID;

static const KinematicsModel kinematics_model = {
    // Leg
    {0.0, 0.055, 0.045, 0.2, 0.2, 0.04, 0.215, 0.13, 0.085, 0.13, 0.02, 2.0 * 0.055, -1, -1, 1, 1, 1, -1},
    // Head
    {-0.007, 0.0, 0.21, 0.048, 0.069, 0, 0.065, M_PI / 90.0, 0.068, -M_PI_4, M_PI_4, -M_PI / 6.0, M_PI / 6.0},
};

static constexpr double ITERATIONS      = 10000;
static constexpr double ERROR_THRESHOLD = 2e-5;


TEST_CASE("Test the Head kinematics", "[utility][motion][kinematics][head]") {
    for (int i = 0; i < ITERATIONS; ++i) {

        // Make a random camera vector
        Eigen::Vector3d camera_vector = Eigen::Vector3d::Random().normalized();

        INFO("Testing with the random vector, " << camera_vector.transpose());

        std::vector<std::pair<ServoID, double>> angles =
            utility::actuation::kinematics::calculate_head_joints(camera_vector);

        // Make our sensors object
        Sensors sensors;
        sensors.servo = std::vector<Sensors::Servo>(20);

        // Insert our known sensors (from the calculated angles) into our sensors
        for (const auto& angle : angles) {
            ServoID servoID;
            double position;

            std::tie(servoID, position) = angle;

            sensors.servo[static_cast<int>(servoID)].present_position = position;
        }

        // Do our forward kinematics
        Eigen::Isometry3d Htc =
            utility::actuation::kinematics::calculatePosition(kinematics_model,
                                                              sensors,
                                                              ServoID::HEAD_PITCH)[ServoID::HEAD_PITCH];

        // Check that our vector that forward kinematics finds is close to what is expected
        REQUIRE_THAT(double(Htc(0, 0) - camera_vector[0]), WithinAbs(0.0, ERROR_THRESHOLD));
        REQUIRE_THAT(double(Htc(1, 0) - camera_vector[1]), WithinAbs(0.0, ERROR_THRESHOLD));
        REQUIRE_THAT(double(Htc(2, 0) - camera_vector[2]), WithinAbs(0.0, ERROR_THRESHOLD));
    }
}

TEST_CASE("Test the Leg kinematics", "[utility][motion][kinematics][leg]") {
    for (int i = 0; i < ITERATIONS; ++i) {

        // Make a random camera vector
        Eigen::Isometry3d ik_request = Eigen::Isometry3d::Identity();
        Eigen::Vector3d rotation     = Eigen::Vector3d::Random() * 2.0 * M_PI;
        ik_request                   = ik_request.rotate(Eigen::AngleAxisd(rotation.x(), Eigen::Vector3d::UnitX()));
        ik_request                   = ik_request.rotate(Eigen::AngleAxisd(rotation.y(), Eigen::Vector3d::UnitY()));
        ik_request                   = ik_request.rotate(Eigen::AngleAxisd(rotation.z(), Eigen::Vector3d::UnitZ()));
        ik_request.translation()     = ((Eigen::Vector3d::Random() + Eigen::Vector3d::Ones()) * 0.5)
                                       .cwiseProduct(Eigen::Vector3d(0.03, 0.03, 0.10));

        INFO("Testing with the random transform, \n" << ik_request.matrix());

        Sensors sensors;
        sensors.servo = std::vector<Sensors::Servo>(20);

        std::vector<std::pair<ServoID, double>> left_leg_joints =
            utility::actuation::kinematics::calculate_leg_joints(kinematics_model, ik_request, LimbID::LEFT_LEG);
        for (const auto& leg_joint : left_leg_joints) {
            ServoID servoID;
            double position;

            std::tie(servoID, position) = leg_joint;

            sensors.servo[servoID].present_position = position;
        }

        std::vector<std::pair<ServoID, double>> right_leg_joints =
            utility::actuation::kinematics::calculate_leg_joints(kinematics_model, ik_request, LimbID::RIGHT_LEG);
        for (const auto& leg_joint : right_leg_joints) {
            ServoID servoID;
            double position;

            std::tie(servoID, position) = leg_joint;

            sensors.servo[servoID].present_position = position;
        }

        INFO("Calculating forward kinematics");
        Eigen::Isometry3d left_foot_position =
            utility::actuation::kinematics::calculatePosition(kinematics_model,
                                                              sensors,
                                                              ServoID::L_ANKLE_ROLL)[ServoID::L_ANKLE_ROLL];
        Eigen::Isometry3d right_foot_position =
            utility::actuation::kinematics::calculatePosition(kinematics_model,
                                                              sensors,
                                                              ServoID::R_ANKLE_ROLL)[ServoID::R_ANKLE_ROLL];
        INFO("Forward Kinematics predicts left foot: \n" << left_foot_position.matrix());
        INFO("Forward Kinematics predicts right foot: \n" << right_foot_position.matrix());
        INFO("Compared to request: \n" << ik_request.matrix());
        for (size_t servo_id = 0; servo_id < ServoID::NUMBER_OF_SERVOS; ++servo_id) {
            INFO(ServoID(servo_id) << ": " << sensors.servo[servo_id].present_position);
        }

        double lerror = (left_foot_position.matrix().array() - ik_request.matrix().array()).abs().maxCoeff();
        double rerror = (right_foot_position.matrix().array() - ik_request.matrix().array()).abs().maxCoeff();

        REQUIRE_THAT(lerror, WithinAbs(0.0, ERROR_THRESHOLD));
        REQUIRE_THAT(rerror, WithinAbs(0.0, ERROR_THRESHOLD));
    }
}
