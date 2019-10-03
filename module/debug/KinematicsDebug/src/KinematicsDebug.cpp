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

#include "KinematicsDebug.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdlib>

#include "extension/Configuration.h"

#include "message/input/Sensors.h"
#include "message/motion/KinematicsModel.h"
#include "message/motion/ServoTarget.h"

#include "utility/behaviour/Action.h"
#include "utility/input/ServoID.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/motion/InverseKinematics.h"

namespace module {
namespace debug {
    using extension::Configuration;

    using message::input::Sensors;
    using message::motion::BodySide;
    using message::motion::KinematicsModel;
    using message::motion::ServoTarget;

    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;
    using utility::motion::kinematics::calculateCameraLookJoints;
    using utility::motion::kinematics::calculateLegJoints;
    using utility::motion::kinematics::calculatePosition;

    KinematicsDebug::KinematicsDebug(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration, With<KinematicsModel>>("InverseKinematicsRequest.yaml")
            .then([this](const Configuration& request, const KinematicsModel& kinematicsModel) {
                return;  // WTF is this?
                Eigen::Affine3d target = Eigen::Affine3d::Identity();
                target =
                    target.rotate(Eigen::AngleAxisd(request.config["xAngle"].as<double>(), Eigen::Vector3d::UnitX()));
                target =
                    target.rotate(Eigen::AngleAxisd(request.config["yAngle"].as<double>(), Eigen::Vector3d::UnitY()));
                target =
                    target.rotate(Eigen::AngleAxisd(request.config["zAngle"].as<double>(), Eigen::Vector3d::UnitZ()));

                // translation
                target.translation() = Eigen::Vector3d(request.config["x"].as<double>(),   // down/up
                                                       request.config["y"].as<double>(),   // left/right
                                                       request.config["z"].as<double>());  // front/back

                bool left  = request.config["left"].as<bool>();
                bool right = request.config["right"].as<bool>();

                auto waypoints = std::make_unique<std::vector<ServoTarget>>();

                if (left) {
                    std::vector<std::pair<ServoID, float>> legJoints =
                        calculateLegJoints(kinematicsModel, target, LimbID::LEFT_LEG);
                    for (auto& legJoint : legJoints) {
                        ServoTarget waypoint;

                        ServoID servoID;
                        float position;

                        std::tie(servoID, position) = legJoint;

                        waypoint.time     = NUClear::clock::now() + std::chrono::seconds(2);
                        waypoint.id       = servoID;
                        waypoint.position = position;
                        waypoint.gain     = 20;

                        waypoints->push_back(waypoint);
                    }
                }

                if (right) {
                    std::vector<std::pair<ServoID, float>> legJoints =
                        calculateLegJoints(kinematicsModel, target, LimbID::RIGHT_LEG);
                    for (auto& legJoint : legJoints) {
                        ServoTarget waypoint;

                        ServoID servoID;
                        float position;

                        std::tie(servoID, position) = legJoint;

                        waypoint.time     = NUClear::clock::now() + std::chrono::seconds(2);
                        waypoint.id       = servoID;
                        waypoint.position = position;
                        waypoint.gain     = 20;

                        waypoints->push_back(waypoint);
                    }
                }

                emit(std::move(waypoints));
            });

        on<Configuration, With<KinematicsModel>>("LegKinematicsNULLTest.yaml")
            .then([this](const Configuration& request, const KinematicsModel& kinematicsModel) {
                int iterations        = 1;
                int numberOfFails     = 0;
                float ERROR_THRESHOLD = request.config["ERROR_THRESHOLD"].as<float>();

                if (request.config["RANDOMIZE"].as<bool>()) {
                    iterations = request.config["RANDOM_ITERATIONS"].as<int>();
                }

                for (int i = 0; i < iterations; i++) {
                    Eigen::Affine3d ikRequest = Eigen::Affine3d::Identity();

                    // translation
                    ikRequest.translation() = Eigen::Vector3d(request.config["x"].as<double>(),
                                                              request.config["y"].as<double>(),
                                                              request.config["z"].as<double>());

                    if (request.config["RANDOMIZE"].as<bool>()) {
                        ikRequest               = ikRequest.rotate(Eigen::AngleAxisd(
                            2.0 * M_PI * rand() / static_cast<double>(RAND_MAX), Eigen::Vector3d::UnitY()));
                        ikRequest               = ikRequest.rotate(Eigen::AngleAxisd(
                            2.0 * M_PI * rand() / static_cast<double>(RAND_MAX), Eigen::Vector3d::UnitX()));
                        ikRequest               = ikRequest.rotate(Eigen::AngleAxisd(
                            2.0 * M_PI * rand() / static_cast<double>(RAND_MAX), Eigen::Vector3d::UnitZ()));
                        ikRequest.translation() = ((Eigen::Vector3d::Random() + Eigen::Vector3d::Ones()) * 0.5)
                                                      .cwiseProduct(Eigen::Vector3d(0.03, 0.03, 0.10));
                    }
                    else {
                        ikRequest = ikRequest.rotate(
                            Eigen::AngleAxisd(request.config["yAngle"].as<double>(), Eigen::Vector3d::UnitY()));
                        ikRequest = ikRequest.rotate(
                            Eigen::AngleAxisd(request.config["xAngle"].as<double>(), Eigen::Vector3d::UnitX()));
                        ikRequest = ikRequest.rotate(
                            Eigen::AngleAxisd(request.config["zAngle"].as<double>(), Eigen::Vector3d::UnitZ()));
                    }

                    bool left  = request.config["left"].as<bool>();
                    bool right = request.config["right"].as<bool>();

                    std::unique_ptr<Sensors> sensors = std::make_unique<Sensors>();
                    sensors->servo                   = std::vector<Sensors::Servo>(20);

                    if (left) {
                        std::vector<std::pair<ServoID, float>> legJoints =
                            calculateLegJoints(kinematicsModel, ikRequest, LimbID::LEFT_LEG);
                        for (auto& legJoint : legJoints) {
                            ServoID servoID;
                            float position;

                            std::tie(servoID, position) = legJoint;

                            sensors->servo[servoID].present_position = position;
                        }
                    }

                    if (right) {
                        std::vector<std::pair<ServoID, float>> legJoints =
                            calculateLegJoints(kinematicsModel, ikRequest, LimbID::RIGHT_LEG);
                        for (auto& legJoint : legJoints) {
                            ServoID servoID;
                            float position;

                            std::tie(servoID, position) = legJoint;

                            sensors->servo[servoID].present_position = position;
                        }
                    }
                    std::cout << "KinematicsNULLTest -calculating forward kinematics." << std::endl;
                    Eigen::Affine3d lFootPosition =
                        calculatePosition(kinematicsModel, *sensors, ServoID::L_ANKLE_ROLL)[ServoID::L_ANKLE_ROLL];
                    Eigen::Affine3d rFootPosition =
                        calculatePosition(kinematicsModel, *sensors, ServoID::R_ANKLE_ROLL)[ServoID::R_ANKLE_ROLL];
                    NUClear::log<NUClear::DEBUG>("Forward Kinematics predicts left foot: \n", lFootPosition.matrix());
                    NUClear::log<NUClear::DEBUG>("Forward Kinematics predicts right foot: \n", rFootPosition.matrix());
                    std::cout << "Compared to request: \n" << ikRequest.matrix() << std::endl;
                    for (size_t servoID = 0; servoID < ServoID::NUMBER_OF_SERVOS; servoID++) {
                        std::cout << ServoID(servoID) << ": " << sensors->servo[servoID].present_position << std::endl;
                    }

                    float lmax_error[2] = {0};
                    float rmax_error[2] = {0};
                    for (int i = 0; i < 4; i++) {
                        for (int j = 0; j < 4; j++) {
                            float lerror = std::abs(lFootPosition(i, j) - ikRequest(i, j));
                            float rerror = std::abs(rFootPosition(i, j) - ikRequest(i, j));
                            if (lerror > lmax_error[0]) {
                                lmax_error[0] = lerror;
                            }
                            if (rerror > rmax_error[0]) {
                                rmax_error[0] = rerror;
                            }

                            // We are not looking at x and y translation
                            if (((i != 0) && (i != 1)) || (j != 3)) {
                                if (lerror > lmax_error[1]) {
                                    lmax_error[1] = lerror;
                                }
                                if (rerror > rmax_error[1]) {
                                    rmax_error[1] = rerror;
                                }
                            }
                        }
                    }

                    std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                    if (lmax_error[1] < ERROR_THRESHOLD) {
                        if (lmax_error[0] >= ERROR_THRESHOLD) {
                            std::cout << "LEFT IK TEST PASSED - IK IS INCONSISTENT WITH FK DUE TO HIP PITCH TRANSLATION"
                                      << std::endl;
                        }
                        else {
                            std::cout << "LEFT IK TEST PASSED" << std::endl;
                        }
                    }
                    else {
                        std::cout << "\n\n\n!!!!!!!!!! LEFT IK TEST FAILED !!!!!!!!!!\n\n\n"
                                  << "     (max_error = " << lmax_error[0] << ", " << lmax_error[1] << ")" << std::endl;
                        numberOfFails++;
                    }
                    std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

                    std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                    if (rmax_error[1] < ERROR_THRESHOLD) {
                        if (rmax_error[0] >= ERROR_THRESHOLD) {
                            std::cout
                                << "RIGHT IK TEST PASSED - IK IS INCONSISTENT WITH FK DUE TO HIP PITCH TRANSLATION"
                                << std::endl;
                        }
                        else {
                            std::cout << "RIGHT IK TEST PASSED" << std::endl;
                        }
                    }
                    else {
                        std::cout << "\n\n\n!!!!!!!!!! RIGHT IK TEST FAILED !!!!!!!!!!\n\n\n"
                                  << "     (max_error = " << rmax_error[0] << ", " << rmax_error[1] << ")" << std::endl;
                        numberOfFails++;
                    }
                    std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

                    sensors->Htw.setIdentity();
                    emit(std::move(sensors));
                }
                std::cout << "IK Leg NULL Test : " << numberOfFails << " Total Failures " << std::endl;
            });

        on<Configuration, With<KinematicsModel>>("HeadKinematicsNULLTest.yaml")
            .then([this](const Configuration& request, const KinematicsModel& kinematicsModel) {
                int iterations        = 1;
                int numberOfFails     = 0;
                float ERROR_THRESHOLD = request.config["ERROR_THRESHOLD"].as<float>();
                float yaw             = request.config["yaw"].as<float>();
                float pitch           = request.config["pitch"].as<float>();
                bool RANDOMIZE        = request.config["RANDOMIZE"].as<bool>();

                Eigen::Vector3d cameraVec(
                    std::cos(yaw) * std::cos(pitch), std::sin(yaw) * std::cos(pitch), -std::sin(pitch));
                if (RANDOMIZE) {
                    iterations = request.config["RANDOM_ITERATIONS"].as<int>();
                }

                for (int i = 0; i < iterations; i++) {
                    if (RANDOMIZE) {
                        cameraVec = ((Eigen::Vector3d::Random() + Eigen::Vector3d::Ones()) * 0.5).normalized();
                    }

                    std::vector<std::pair<ServoID, float>> angles =
                        calculateCameraLookJoints(kinematicsModel, cameraVec);
                    Sensors sensors;
                    sensors.servo = std::vector<Sensors::Servo>(20);

                    for (auto& angle : angles) {
                        ServoID servoID;
                        float position;

                        std::tie(servoID, position) = angle;

                        sensors.servo[servoID].present_position = position;
                    }

                    Eigen::Affine3d fKin =
                        calculatePosition(kinematicsModel, sensors, ServoID::HEAD_PITCH)[ServoID::HEAD_PITCH];

                    float max_error = 0;
                    for (int i = 0; i < 3; i++) {
                        float error = std::abs(fKin(i, 0) - cameraVec[i]);
                        if (error > max_error) {
                            max_error = error;
                        }
                    }
                    if (max_error >= ERROR_THRESHOLD) {
                        numberOfFails++;
                    }

                    NUClear::log<NUClear::DEBUG>("++++++++++++++++++++++++++++++++++++++++++++++++++");
                    NUClear::log<NUClear::DEBUG>("Request = \n", cameraVec);
                    NUClear::log<NUClear::DEBUG>("Angles = \n", angles[0].second, angles[1].second);
                    NUClear::log<NUClear::DEBUG>("Final FKin = \n", fKin.matrix());
                    NUClear::log<NUClear::DEBUG>(max_error >= ERROR_THRESHOLD ? "FAIL" : "PASS",
                                                 "Max Error =",
                                                 max_error,
                                                 ", ERROR_THRESHOLD",
                                                 ERROR_THRESHOLD);
                    NUClear::log<NUClear::DEBUG>("++++++++++++++++++++++++++++++++++++++++++++++++++");
                }
                std::cout << "IK Head NULL Test : " << numberOfFails << " Total Failures out of " << iterations
                          << std::endl;
            });
    }
}  // namespace debug
}  // namespace module
